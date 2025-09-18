#define MM2M_NAME       "mm2m"

#ifndef pr_fmt
#define pr_fmt(fmt)     MM2M_NAME ": " fmt
#endif

#include <linux/platform_device.h>
#include <linux/printk.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-vmalloc.h>

#define MAX_HEIGHT                    (4320)
#define MAX_WIDTH                     (7680)

/**
 * @brief: 最小m2m设备的设备对象
 */
struct mm2m_dev
{
    struct v4l2_device      v4l2_dev;          /** 基础v4l2容器 */
    struct video_device     vfd;               /** video设备 */
    struct v4l2_m2m_dev     *m2m_dev;          /** m2m上下文控制器 */

    struct workqueue_struct *workqueue;        /** 由于本驱动未设置帧率限制，因此若使用共享队列则会占用过多的共享资源 */
};

/**
 * @brief: 上下文实例，在open回调中被创建
 */
struct mm2m_ctx
{
    struct v4l2_fh          fh;                 /** V4L2的通用文件管理句柄，用于V4L2内部管理用户态的上下文实例 */
    struct mm2m_dev         *dev;               /** 驱动私有数据，会在 `device_work` 中用于提交已完成的任务 */

    struct delayed_work     work;               /** 可延迟工作对象，用于在device_run中异步执行帧复制任务 */

    // 输入方向的配置信息，输出方向和输入方向保持一致
    int                     curr_fmt_index;     /** 当前设置的像素格式 */
    uint32_t                curr_width;         /** 当前设置的分辨率宽度 */
    uint32_t                curr_height;        /** 当前设置的分辨率高度 */
    atomic_t                running;            /** 任务运行状态标志位，当其为0时表示任务结束 */
};

/**
 * @brief: 从文件指针中获取最小m2m设备的上下文实例
 * @param file: fops传来的文件指针
 * @return: 上下文实例指针
 */
static inline struct mm2m_ctx *file2ctx(struct file *file)
{
    return container_of(file->private_data, struct mm2m_ctx, fh);
}

/**
 * @brief: 从延迟工作对象获取最小m2m设备的上下文实例
 * @param work: 延迟工作对象指针
 * @return: 上下文实例指针
 */
static inline struct mm2m_ctx *work2ctx(struct work_struct* work)
{
    return container_of(work, struct mm2m_ctx, work.work);
}

/**
 * @brief: 单个像素格式的相关基础信息
 */
struct mm2m_fmt {
    uint32_t pix_format;
    int      pix_size;
};

/**
 * @brief: 选取常见的若干种像素格式
 */
static struct mm2m_fmt formats[] =
{
    {
        .pix_format = V4L2_PIX_FMT_RGB24,  /* 24-bit RGB 8:8:8 */
        .pix_size   = 3,
    },
    {
        .pix_format = V4L2_PIX_FMT_BGR24,  /* 24-bit BGR 8:8:8 */
        .pix_size   = 3,
    },
    {
        .pix_format = V4L2_PIX_FMT_RGB565, /* 16-bit RGB 5:6:5 */
        .pix_size   = 2,
    },
    {
        .pix_format = V4L2_PIX_FMT_RGB32,  /* 32-bit RGB 8:8:8:8 (XRGB) */
        .pix_size   = 4,
    },
    {
        .pix_format = V4L2_PIX_FMT_BGR32,  /* 32-bit BGR 8:8:8:8 (XBGR) */
        .pix_size   = 4,
    },
    {
        .pix_format = V4L2_PIX_FMT_GREY,   /* 8-bit Greyscale */
        .pix_size   = 1,
    },
    {
        .pix_format = V4L2_PIX_FMT_YUYV,   /* 16-bit YUV 4:2:2 packed (Y-U-Y-V) */
        .pix_size   = 2,
    },
    {
        .pix_format = V4L2_PIX_FMT_UYVY,   /* 16-bit YUV 4:2:2 packed (U-Y-V-Y) */
        .pix_size   = 2,
    },
};

/**
 * @brief: 找到pix_format对应的引索
 * @param pix_format: 目标格式
 * @return: 当找到目标pix_format时返回引索，否则返回-1
 */
static int find_format_index(uint32_t pix_format)
{
    for(int i = 0; i < ARRAY_SIZE(formats); i++)
    {
        if(formats[i].pix_format == pix_format)
            return i;
    }
    return -1;
}


/**
 * @brief: 驱动处理m2m作业的入口，作业不需要在该回调返回前结束
 * @param priv
 */
static void device_run(void *priv)
{
    // 本函数的参数priv是v4l2_m2m_ctx_init时参数drv_priv的值，即mm2m_ctx对应指针
    struct mm2m_ctx *ctx = priv;
    //struct vb2_v4l2_buffer *src_buf, *dst_buf;

    // v4l2_info(&ctx->dev->v4l2_dev, "drvice running..");

    // 将ctx实例中的工作对象注册到共享工作队列中
    queue_delayed_work(ctx->dev->workqueue, &ctx->work, 0);
}

/**
 * @brief: 查询设备能否立即开启新任务
 * @param priv
 * @return: 返回1表示已准备好
 */
static int job_ready(void *priv)
{
    struct mm2m_ctx *ctx = priv;
    if(ctx->curr_width && ctx->curr_height)
    {
        // v4l2_info(&ctx->dev->v4l2_dev, "job_ready return 1");
        return 1;
    } else {
        // v4l2_info(&ctx->dev->v4l2_dev, "job_ready return 0");
        return 0;
    }
}

/**
 * @brief: 紧急停止任务
 * @param priv
 */
static void job_abort(void *priv)
{
    struct mm2m_ctx *ctx = priv;

    // 修改运行状态，结束设备运行
    atomic_dec(&ctx->running);
}

/**
 * @brief: 延迟任务回调函数
 * @param w
 */
static void device_work(struct work_struct *w)
{
    // 获取当前ctx
    struct mm2m_ctx* ctx = work2ctx(w);
    struct vb2_v4l2_buffer *src_buf, *dst_buf;

    // v4l2_info(&ctx->dev->v4l2_dev, "drvice working..");

    // 1. 获取缓冲区
    src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
    dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

    // 2. 拷贝数据
    memcpy(
        vb2_plane_vaddr(&dst_buf->vb2_buf, 0),
        vb2_plane_vaddr(&src_buf->vb2_buf, 0),
        dst_buf->vb2_buf.planes[0].length
    );

    // 3. 设置已用数据长度
    if (src_buf->vb2_buf.planes[0].bytesused) {
        vb2_set_plane_payload(&dst_buf->vb2_buf, 0, src_buf->vb2_buf.planes[0].bytesused);
    } else {
        vb2_set_plane_payload(&dst_buf->vb2_buf, 0, src_buf->vb2_buf.planes[0].length);
    }

    // 4. 标记缓冲区完成
    v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
    v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

    // 5. 通知当前job完成，随后m2m框架才会开启下一个任务
    v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx);
}


/**
 * @brief: 队列配置回调
 * @param vq: 需要配置的vb2缓冲区指针
 * @param num_buffers: 驱动所需要的缓冲区数量
 * @param num_planes: 驱动所需平面数量
 * @param sizes: 存储每个平面所需字节数
 * @param alloc_devs: 存储每个平面所分配的设备
 * @return: 0 if success.
 */
static int queue_setup(struct vb2_queue *vq, unsigned int *num_buffers, unsigned int *num_planes,
                       unsigned int sizes[], struct device *alloc_devs[])
{
    // 获取在 `queue_init` 时寄存的ctx数据
    struct mm2m_ctx *ctx = vq->drv_priv;

    // 核校参数
    if(ctx->curr_height == 0 || ctx->curr_width == 0)
        return -EINVAL;

    if(*num_planes == 0) {
        // 首次调用，此时驱动进行参数配置
        // 配置为单平面
        *num_planes = 1;
        // 每个平面的总字节数为 w * h * pix_size
        sizes[0] = ctx->curr_height * ctx->curr_width * formats[ctx->curr_fmt_index].pix_size;

        // 确保缓冲区数量，此时 `num_buffers` 会介于 [0, VIDEO_MAX_FRAME] 之间
        if(*num_buffers < 3) {
            *num_buffers = 3;
        }

        v4l2_info(&ctx->dev->v4l2_dev,
                  "Set num_planes=%d, size=%d, num_buffers=%d\n", *num_planes, sizes[0], *num_buffers);
    } else {
        // 后续调用，校验参数是否正确
        if(*num_planes != 1)
        {
            return -EINVAL;
        }

        if(sizes[0] != ctx->curr_height * ctx->curr_width * formats[ctx->curr_fmt_index].pix_size)
        {
            return -EINVAL;
        }
    }
    return 0;
}


/**
 * @brief: 用户态将缓冲区添加到队列后的回调
 * @param vb: 用户所添加的缓冲区
 */
static void buffer_queue(struct vb2_buffer *vb)
{
    struct mm2m_ctx *ctx = vb->vb2_queue->drv_priv;
    // 调用 `v4l2_m2m_buf_queue` 将用户入队的缓冲区添加到对应的就绪队列中
    v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}


/**
 * @brief: 启动流传输回调。
 * @param q:
 * @param count: 执行该回调时已经入队的缓冲区数量
 * @return
 */
static int start_streaming(struct vb2_queue *q, unsigned int count)
{
    struct mm2m_ctx *ctx = q->drv_priv;

    v4l2_info(&ctx->dev->v4l2_dev, "try to start_steaming\n");

    // 启用m2m
    atomic_inc(&ctx->running);
    return 0;
}


/**
 * @brief: 停止流传输回调
 * @param q: 所停止的vb2队列
 */
static void stop_streaming(struct vb2_queue *q)
{
    struct mm2m_ctx *ctx = q->drv_priv;
    struct vb2_v4l2_buffer *vbuf = NULL;

    atomic_dec(&ctx->running);

    while(1) {
        if (V4L2_TYPE_IS_OUTPUT(q->type))
            vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
        else
            vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
        if (!vbuf)
            return;
        v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
    }
}


/**
 * @brief:
 */
static const struct v4l2_m2m_ops m2m_ops = {
    .device_run	= device_run,
    .job_ready	= job_ready,
    .job_abort	= job_abort,
};



static const struct vb2_ops mm2m_qops = {
    .queue_setup     = queue_setup,           /** 队列配置回调 */
    .buf_queue       = buffer_queue,          /** 用户态将缓冲区入队后的回调 */
    .start_streaming = start_streaming,       /** 启动流传输回调 */
    .stop_streaming  = stop_streaming,        /** 停止流传输回调 */
};



/**
 * @brief: 队列初始化回调
 * @param priv: 指向驱动私有数据，其值为 `v4l2_m2m_ctx_init` 传入的 `drv_priv` 参数(第二个参数)
 * @param src_vq: 输入缓冲区，数据流向为 用户->内核，其常被设置为 `V4L2_BUF_TYPE_VIDEO_OUTPUT` 类型
 * @param dst_vq: 输出缓冲区，数据流向为 内核->用户，其常被设置为 `V4L2_BUF_TYPE_VIDEO_CAPTURE` 类型
 * @return
 */
static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
    struct mm2m_ctx *ctx = priv;
    int ret = 0;

    // 设置输入缓冲区，数据流向为 用户->内核
    src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
    src_vq->drv_priv = ctx;
    src_vq->ops = &mm2m_qops;
    src_vq->mem_ops = &vb2_vmalloc_memops;                        // 指定使用vmalloc内存
    src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    // 这里需要定义vb2所需的内存，从而使用拓展的 `v4l2_m2m_buffer` 。
    // 不然 `buffer_queue` 回调中无法使用m2m提供的 `v4l2_m2m_buf_queue` 方法而需要自行实现。
    src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
    ret = vb2_queue_init(src_vq);
    if (ret)
        return ret;

    // 设置输出缓冲区，数据流向为 内核->用户
    dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
    dst_vq->drv_priv = ctx;
    dst_vq->ops = &mm2m_qops;
    dst_vq->mem_ops = &vb2_vmalloc_memops;
    dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
    ret = vb2_queue_init(dst_vq);

    return ret;
}


/**
 * @brief: video_device打开回调，需要在此创建设备上下文
 * @param file: 文件指针
 * @return: 0 if success
 */
static int mm2m_open(struct file *file)
{
    int ret = 0;
    // 从文件指针获取设备指针
    struct mm2m_dev *dev = video_drvdata(file);
    struct mm2m_ctx *ctx = NULL;

    // 分配上下文实例
    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if(!ctx) {
        ret = -ENOMEM;
        goto err_alloc_ctx;
    }

    // 初始化文件句柄
    v4l2_fh_init(&ctx->fh, video_devdata(file));

    // 将驱动私有数据寄存到上下文中
    ctx->dev = dev;

    // 将上下文的文件句柄存入private_data
    // 注意不是存自己定义的上下文实例，因为V4L2内部实现是依赖于其文件句柄
    file->private_data = &ctx->fh;

    // 初始化m2m上下文实例
    ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);

    // 将文件句柄添加到设备列表
    v4l2_fh_add(&ctx->fh);

    // 初始化延迟工作对象，并设置回调函数
    INIT_DEFERRABLE_WORK(&ctx->work, device_work);

err_alloc_ctx:
    return ret;
}

/**
 * @brief: 设备每次open得到的fd被完全关闭时触发，即打开计数器归0时触发
 * @param file: 文件指针
 * @return: 0 if success
 */
static int mm2m_release(struct file *file)
{
    struct mm2m_ctx *ctx = file2ctx(file);

    // 取消所有任务
    cancel_delayed_work_sync(&ctx->work);
    // 确保流传输停止
    if (atomic_read(&ctx->running)) {
        struct vb2_queue *src_vq, *dst_vq;

        src_vq = v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
        dst_vq = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);

        if (vb2_is_streaming(src_vq))
            vb2_queue_error(src_vq);

        if (vb2_is_streaming(dst_vq))
            vb2_queue_error(dst_vq);

        atomic_set(&ctx->running, 0);
    }

    // 倒序释放资源
    v4l2_fh_del(&ctx->fh);
    v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
    v4l2_fh_exit(&ctx->fh);
    kfree(ctx);
    return 0;
}

/**
 * @brief: video_device的文件回调函数
 */
static const struct v4l2_file_operations mm2m_fops = {
    .owner          = THIS_MODULE,
    .open           = mm2m_open,                        /** 设备打开回调函数 */
    .release        = mm2m_release,                     /** 设备释放回调函数 */
    .poll           = v4l2_m2m_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap           = v4l2_m2m_fop_mmap,
};

/**
 * @brief: 查询设备能力回调(ioctl(VIDIOC_QUERYCAP, ...))
 */
static int vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
    strscpy(cap->driver, MM2M_NAME, sizeof(cap->driver));
    strscpy(cap->card, MM2M_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info),
             "platfrom:%s", MM2M_NAME);
    return 0;
}

/**
 * @brief: 枚举输入方向(video_capture)支持的数据格式
 * @note:
 * @param file:
 * @param fh:
 * @param f:
 * @return
 */
static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
    struct mm2m_ctx *ctx = fh;
    struct mm2m_dev *dev = ctx->dev;
    v4l2_info(&dev->v4l2_dev, "enum videoc cap fmt at index:%d\n", f->index);

    // 对于输出设备，其只支持与输入相同的类型
    if(f->index == 0 && ctx->curr_fmt_index >= 0)
    {
        f->pixelformat = formats[ctx->curr_fmt_index].pix_format;
        return 0;
    } else {
        return -EINVAL;
    }
}


/**
 * @brief: 枚举输出方向支持的数据格式
 * @param file
 * @param fh
 * @param f
 * @return
 */
static int vidioc_enum_fmt_vid_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
    struct mm2m_ctx *ctx = fh;
    struct mm2m_dev *dev = ctx->dev;
    v4l2_info(&dev->v4l2_dev, "enum videoc output fmt at index:%d\n", f->index);

    // 对于输入设备(用于->内核)，支持所有已知类型
    if(f->index < ARRAY_SIZE(formats))
    {
        f->pixelformat = formats[f->index].pix_format;
        return 0;
    } else {
        // 枚举结束
        return -EINVAL;
    }
}


/**
 * @brief: 查询capture和output的当前格式
 * @note:
 * @param file: 用户空间操作的文件句柄，正常驱动应当从中获取上下文实例
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_g_fmt_vid_cap_out(struct file *file, void *fh, struct v4l2_format *f)
{
    struct mm2m_ctx *ctx    = fh;

    f->fmt.pix.width        = ctx->curr_width;
    f->fmt.pix.height       = ctx->curr_height;
    f->fmt.pix.field        = V4L2_FIELD_NONE;                          /** 使用普通逐行扫描，大多数CMOS均为此格式 */
    f->fmt.pix.pixelformat  = formats[ctx->curr_fmt_index].pix_format;
    return 0;
}

/**
 * @brief: 设置video_capture的数据格式
 * @note: 除非 `f->type` 字段错误，否则不应返回错误代码
 * @param file: 用户空间操作的文件句柄
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    // struct mm2m_ctx* ctx = fh;

    // 文档规定：除非 `f->type` 字段错误，否则不应返回错误代码
    if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    // 由于输出格式完全取决于输入格式，因此实际上本回调等价于查询当前格式的回调。
    return vidioc_g_fmt_vid_cap_out(file, fh, f);
}

/**
 * @brief: 设置video_output的数据格式
 * @note: 除非 `f->type` 字段错误，否则不应返回错误代码
 * @param file: 用户空间操作的文件句柄
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_s_fmt_vid_out(struct file *file, void *fh, struct v4l2_format *f)
{
    struct mm2m_ctx* ctx = fh;
    struct mm2m_dev *dev = ctx->dev;

    // 文档规定：除非 `f->type` 字段错误，否则不应返回错误代码
    if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
        return -EINVAL;

    // 查找目标格式对应引索，大于等于0时为引索，否则失败
    int index = find_format_index(f->fmt.pix.pixelformat);
    if (index < 0) {
        index = 0;
    }

    // 修正目标格式
    f->fmt.pix.width        = min(max(f->fmt.pix.width, 1), MAX_WIDTH);
    f->fmt.pix.height       = min(max(f->fmt.pix.height, 1), MAX_HEIGHT);
    f->fmt.pix.field        = V4L2_FIELD_NONE;                                    /** 使用普通逐行扫描，大多数CMOS均为此格式 */
    f->fmt.pix.pixelformat  = formats[index].pix_format;

    // 同步目标格式
    ctx->curr_fmt_index = index;
    ctx->curr_width     = f->fmt.pix.width;
    ctx->curr_height    = f->fmt.pix.height;

    // 输出目标格式到日志
    v4l2_info(&dev->v4l2_dev, "Set video output format to %c.%c.%c.%c, %dx%d",
        formats[index].pix_format >> 0  & 0xff,
        formats[index].pix_format >> 8  & 0xff,
        formats[index].pix_format >> 16 & 0xff,
        formats[index].pix_format >> 24 & 0xff,
        f->fmt.pix.width, f->fmt.pix.height
    );

    return 0;
}


/**
 * @brief: 枚举分辨率的回调函数(ioctl(VIDIOC_ENUM_FRAMESIZES))
 * @param file:
 * @param fh:
 * @param fsize: 返回给用户空间的分辨率信息
 * @note: fsize->type并非是 `V4L2_BUF_TYPE_VIDEO_OUTPUT` 等数据流向指示，而是指定分辨率类型( `V4L2_FRMSIZE_TYPE_CONTINUOUS` 等)
 * @return
 */
static int vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
    struct mm2m_ctx *ctx = fh;

    // 验证请求的像素格式在支持列表内
    if(find_format_index(fsize->pixel_format) < 0)
        return -EINVAL;

    // 仅使用一个连续分辨率
    if (fsize->index != 0)
        return -EINVAL;

    // 填充分辨率
    fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
    fsize->stepwise.min_width  = 1;
    fsize->stepwise.max_width  = MAX_WIDTH;
    fsize->stepwise.step_width = 1;
    fsize->stepwise.min_height = 1;
    fsize->stepwise.max_height = MAX_HEIGHT;
    fsize->stepwise.step_height= 1;

    return 0;
}


static const struct v4l2_ioctl_ops mm2m_ioctl_ops = {
    .vidioc_querycap         = vidioc_querycap,          /** 查询设备能力 */
    .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,  /** 枚举video_capture支持的格式(ioctl(VIDIOC_ENUM_FMT)) */
    .vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,  /** 枚举video_output支持的格式(ioctl(VIDIOC_ENUM_FMT)) */
    .vidioc_g_fmt_vid_cap    = vidioc_g_fmt_vid_cap_out, /** 查询capture和output的当前格式 */
    .vidioc_g_fmt_vid_out    = vidioc_g_fmt_vid_cap_out,
    .vidioc_s_fmt_vid_cap    = vidioc_s_fmt_vid_cap,     /** 设置video_capture的数据格式 */
    .vidioc_s_fmt_vid_out    = vidioc_s_fmt_vid_out,     /** 设置video_output的数据格式 */
    .vidioc_enum_framesizes  = vidioc_enum_framesizes,   /** 枚举分辨率 */

    .vidioc_reqbufs		     = v4l2_m2m_ioctl_reqbufs,
    .vidioc_querybuf	     = v4l2_m2m_ioctl_querybuf,
    .vidioc_qbuf		     = v4l2_m2m_ioctl_qbuf,
    .vidioc_dqbuf		     = v4l2_m2m_ioctl_dqbuf,
    .vidioc_prepare_buf	     = v4l2_m2m_ioctl_prepare_buf,
    .vidioc_create_bufs	     = v4l2_m2m_ioctl_create_bufs,
    .vidioc_expbuf		     = v4l2_m2m_ioctl_expbuf,
    .vidioc_streamon         = v4l2_m2m_ioctl_streamon,
    .vidioc_streamoff        = v4l2_m2m_ioctl_streamoff,
};


/**
 * @brief: video设备被注销时的资源释放回调
 * @note:
 *      触发流程：mm2m_exit -> platform_driver_unregister -> mm2m_remove -> video_unregister_device -> mm2m_video_release
 *      资源管理原则：谁注册谁释放，video_device并没有注册资源，所以不需要释放资源
 * @param vdev
 */
static void mm2m_video_release(struct video_device *vdev)
{
    //struct mm2m_dev *dev = container_of(vdev, struct mm2m_dev, vfd);
}


/**
 * @brief: video_device的基本配置
 */
static const struct video_device mm2m_videodev = {
    .name        = MM2M_NAME,                                /** 暴露于用户空间的设备名称 */
    .vfl_dir     = VFL_DIR_M2M,                              /** 设备数据流向指定为内存到内存 */
    .fops        = &mm2m_fops,
    .ioctl_ops   = &mm2m_ioctl_ops,
    .release     = mm2m_video_release,
    .device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING,
};

/**
 * @brief: 平台设备的探测回调
 * @note: 在本函数中需要对平台设备进行初始化，即初始化m2m设备
 * @param pdev: 需要探测的平台设备
 * @return: 0 if success
 */
static int mm2m_probe(struct platform_device *pdev)
{
    int ret = 0;

    // 1. 为设备对象分配内存
    struct mm2m_dev *dev = kzalloc(sizeof(struct mm2m_dev), GFP_KERNEL);
    if(dev == NULL)
    {
        ret = -ENOMEM;
        goto err_alloc_mm2m;
    }
    // 1.2 向平台设备内寄存驱动私有数据
    platform_set_drvdata(pdev, dev);

    // 2. 注册v4l2设备
    ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
    if(ret != 0)
    {
        goto err_v4l2_register;
    }

    // 3. 注册video_device
    // 3.1 配置vfd成员
    dev->vfd = mm2m_videodev;
    dev->vfd.v4l2_dev = &dev->v4l2_dev;
    // 3.2 向video_device中寄存驱动私有数据
    video_set_drvdata(&dev->vfd, dev);
    // 3.3 注册video设备
    ret = video_register_device(&dev->vfd, VFL_TYPE_VIDEO, 0);
    if(ret) {
        goto err_video_register;
    }

    // 4. 初始化m2m设备上下文
    dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
    if (IS_ERR(dev->m2m_dev)) {
        goto err_m2m_init;
    }

    // 5. 初始化延迟工作队列
    dev->workqueue = create_singlethread_workqueue("mm2m workqueue");
    if(dev->workqueue == NULL)
    {
        goto err_work_queue;
    }

    return ret;

// next err:
    destroy_workqueue(dev->workqueue);

err_work_queue:
    v4l2_m2m_release(dev->m2m_dev);

err_m2m_init:
    video_unregister_device(&dev->vfd);

err_video_register:
    v4l2_device_unregister(&dev->v4l2_dev);

err_v4l2_register:
    kfree(dev);

err_alloc_mm2m:
    return ret;
}

/**
 * @brief: 平台设备的设备移除函数
 * @note:
 *      触发流程： mm2m_exit -> platform_driver_unregister -> mm2m_remove
 *      资源管理原则：谁注册谁释放，因此需要依次释放：
 *      1. workqueue
 *      2. m2m实例
 *      3. video_device
 *      4. v4l2顶层容器
 *      5. mm2m_dev对象
 * @param pdev
 */
static void mm2m_remove(struct platform_device *pdev)
{
    // 获取驱动私有数据
    struct mm2m_dev *dev = platform_get_drvdata(pdev);

    // 1. 释放独有工作队列
    destroy_workqueue(dev->workqueue);
    // 2. 释放m2m实例
    v4l2_m2m_release(dev->m2m_dev);
    // 3. 释放video_device
    video_unregister_device(&dev->vfd);
    // 4. v4l2顶层容器
    v4l2_device_unregister(&dev->v4l2_dev);
    // 5. 释放mm2m_dev对象
    kfree(dev);
}

/**
 * @brief: 静态管理的平台驱动。驱动在内核中只能是单例模式。
 */
static struct platform_driver mm2m_pdrv =
{
    .probe      = mm2m_probe,
    .remove_new = mm2m_remove,
    .driver     = {
        .name   = MM2M_NAME,             /* 驱动和设备之间通过相同名称进行匹配 */
    },
};

/**
 * @brief: 平台设备引用计数器归零时的释放回调函数
 * @note: 在本回调中只需要释放仅与设备相关的资源，即不需要释放任何资源
 * @param dev: 需要释放的设备
 */
static void mm2m_pdev_release(struct device *dev)
{
    pr_info("mm2m device released.\n");
}

/**
 * @brief: 静态管理的平台设备，对于虚拟设备通常都使用静态注册的单例模式。
 * @note: 因为是虚拟设备，所以若需要改为多m2m的多例模式，也通常是只使用一个平台设备，然后在probe时将m2m多例化
 */
static struct platform_device mm2m_pdev = {
    .name        = MM2M_NAME,                 /** 驱动和设备之间通过相同名称进行匹配 */
    .dev.release = mm2m_pdev_release,         /** 平台设备引用计数器归零时的回调函数 */
};

/**
 * @brief: 模块卸载时的退出函数
 */
static void __exit mm2m_exit(void)
{
    platform_driver_unregister(&mm2m_pdrv);
    platform_device_unregister(&mm2m_pdev);
}

/**
 * @brief: 模块注册时的初始化函数
 * @note: 在该函数中主要负责注册平台设备及平台驱动，具体的v4l2及m2m功能会在probe函数中初始化
 */
static int __init mm2m_init(void)
{
    int ret;

    ret = platform_device_register(&mm2m_pdev);
    if (ret)
        return ret;

    ret = platform_driver_register(&mm2m_pdrv);
    if (ret)
        platform_device_unregister(&mm2m_pdev);

    return ret;
}

MODULE_LICENSE("GPL v2");
module_init(mm2m_init);
module_exit(mm2m_exit);
