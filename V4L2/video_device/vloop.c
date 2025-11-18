#define VLOOP_NAME       "vloop"

#ifndef pr_fmt
#define pr_fmt(fmt)     VLOOP_NAME ": " fmt
#endif

#include <linux/kfifo.h>
#include <linux/printk.h>
#include <linux/platform_device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>


#define MAX_HEIGHT                    (4320)
#define MAX_WIDTH                     (7680)
#define FIFO_SIZE                     (VIDEO_MAX_FRAME)      /** 必须为2的整数幂，且大于等于2 */

struct vloop_dev
{
    struct v4l2_device      v4l2_dev;                        /** 基础v4l2容器 */
    struct video_device     vfd;                             /** video设备 */

    struct workqueue_struct *workqueue;                      /** 由于本驱动未设置帧率限制，因此若使用共享队列则会占用过多的共享资源 */
    struct delayed_work     work;                            /** 可延迟工作对象，用于在device_run中异步执行帧复制任务 */

    struct vb2_queue        cap_queue;                       /** 存储输出方向提供的已填充数据的缓冲区 */
    struct vb2_queue        out_queue;                       /** 存储输入方向提供的已读取数据的缓冲区 */

    /* 使用DECLARE_KFIFO时就已经为该成员声明了要使用的缓冲区大小，即对应缓冲区会随着vloop_dev一起被分配 */
    DECLARE_KFIFO(cap_fifo, struct vb2_buffer *, FIFO_SIZE); /** 存储用户传来的输入方向已提交给驱动的缓冲区，状态为ACTIVE */
    DECLARE_KFIFO(out_fifo, struct vb2_buffer *, FIFO_SIZE); /** 存储用户传来的输出方向已提交给驱动的缓冲区，状态为ACTIVE */

    // 输入方向的配置信息，输出方向和输入方向保持一致
    int                     curr_fmt_index;                  /** 当前设置的像素格式 */
    uint32_t                curr_width;                      /** 当前设置的分辨率宽度 */
    uint32_t                curr_height;                     /** 当前设置的分辨率高度 */

    atomic_t                output_running;                  /** 标记输出状态，为0时表示当前设备还没有输出方向数据 */
    atomic_t                capture_count;                   /** capture方向的打开次数 */

    struct mutex		    dev_mutex;                       /** 为了避免编译器的指令重排和CPU的乱序执行，依旧需要使用互斥锁 */
};


/**
 * @brief: 上下文实例，在open回调中被创建
 */
struct vloop_ctx
{
    struct v4l2_fh          fh;                  /** V4L2的通用文件管理句柄，用于V4L2内部管理用户态的上下文实例 */
    struct vloop_dev        *dev;
    struct vb2_queue        *curr_queue;         /** 当前打开方向所使用的vb2队列 */
};


/**
 * @brief: 从文件指针中获取最小m2m设备的上下文实例
 * @param file: fops传来的文件指针
 * @return: 上下文实例指针
 */
static inline struct vloop_ctx *file2ctx(struct file *file)
{
    return container_of(file->private_data, struct vloop_ctx, fh);
}

/**
 * @brief: 从延迟工作对象获取设备指针
 * @param work: 延迟工作对象指针
 * @return: 设备指针
 */
static inline struct vloop_dev *work2dev(struct work_struct* work)
{
    return container_of(work, struct vloop_dev, work.work);
}

/**
 * @brief: 单个像素格式的相关基础信息
 */
struct vloop_fmt {
    uint32_t pix_format;
    int      pix_size;
};

/**
 * @brief: 选取常见的若干种像素格式
 */
static struct vloop_fmt formats[] =
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
 * @brief: 延迟任务回调函数
 * @param w
 */
static void device_work(struct work_struct *w)
{
    // 获取当前dev
    struct vloop_dev* dev = work2dev(w);

    // 校验设备是否仍在运行
    if(!atomic_read(&dev->output_running))
        return;

    // 获取互斥锁
    if (mutex_lock_interruptible(&dev->dev_mutex))
    {
        v4l2_err(&dev->v4l2_dev, "failed to lock mutex.");
        return;
    }

    // 计算可以执行复制的缓冲区数
    int buffer_num = kfifo_len(&dev->cap_fifo) < kfifo_len(&dev->out_fifo) ?
            kfifo_len(&dev->cap_fifo) : kfifo_len(&dev->out_fifo);

    struct vb2_buffer *cap_buffers[FIFO_SIZE] = { NULL };
    struct vb2_buffer *out_buffers[FIFO_SIZE] = { NULL };

    // 将可执行复制的缓冲区出队
    int num = 0;  // 无实际作用，屏蔽 __must_check
    num = kfifo_out(&dev->cap_fifo, cap_buffers, buffer_num);
    num = kfifo_out(&dev->out_fifo, out_buffers, buffer_num);

    // 执行复制
    for(int i = 0; i < buffer_num; i++)
    {
        // 读取已用长度
        size_t out_buf_used = out_buffers[i]->planes[0].bytesused ?
                out_buffers[i]->planes[0].bytesused : out_buffers[i]->planes[0].length;
        // 拷贝数据
        memcpy(
            vb2_plane_vaddr(cap_buffers[i], 0),
            vb2_plane_vaddr(out_buffers[i], 0),
            out_buf_used
        );
        // 设置已用数据长度
        vb2_set_plane_payload(cap_buffers[i], 0, out_buf_used);
        // 同步时间戳
        if (out_buffers[i]->timestamp == 0) {
            cap_buffers[i]->timestamp = ktime_get_ns();
        } else {
            cap_buffers[i]->timestamp = out_buffers[i]->timestamp;
        }
        // 标记缓冲区处理完成
        vb2_buffer_done(cap_buffers[i], VB2_BUF_STATE_DONE);
        vb2_buffer_done(out_buffers[i], VB2_BUF_STATE_DONE);
    }

    mutex_unlock(&dev->dev_mutex);
}


/**
 * @brief: 队列配置回调
 * @param vq: 需要配置的vb2缓冲区指针
 * @param num_buffers: 驱动所需要的缓冲区数量
 * @param num_planes: 驱动所需平面数量
 * @param sizes: 存储每个平面所需字节数
 * @param alloc_devs: 存储每个平面所分配的设备
 * @return:
 *      - `0`:       成功
 *      - `-EINVAL`: 参数非法
 *      - `-EBUSY`:  已存在输出缓冲区
 *      - `-EAGAIN`: 当前还未配置输出缓冲区及参数，不可初始化输出缓冲区(用户态会返回 `Resource temporarily unavailable.` )
 * @note: 尽管v4l2并不会单独获取 `struct video_device.lock` ，但是该函数会被 `vidioc_reqbufs` 调用，从而间接得锁
 */
static int queue_setup(struct vb2_queue *vq, unsigned int *num_buffers, unsigned int *num_planes,
                       unsigned int sizes[], struct device *alloc_devs[])
{
    // 获取在 `queue_init` 时寄存的dev指针
    struct vloop_dev *dev = vq->drv_priv;

    // 核校参数
    if(dev->curr_height == 0 || dev->curr_width == 0)
        return -EINVAL;

    if(*num_planes == 0) {
        // 首次调用，此时驱动进行参数配置
        if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
        {
            // 当创建输出方向的队列时，视作**敲定**输出参数和输出数据源
            // 检验是否已存在输出数据源
            if(atomic_read(&dev->output_running))
                return -EBUSY;

            // 标记输出设备已开启
            atomic_inc(&dev->output_running);
        } else if(vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            // 当配置输入方向队列时，需要确保已经确定输出参数，否则返回 `-EAGAIN`
            if(!atomic_read(&dev->output_running))
                return -EAGAIN;

            // 增加一次reader的引用计数器
        }

        // 配置为单平面
        *num_planes = 1;
        // 每个平面的总字节数为 w * h * pix_size
        sizes[0] = dev->curr_height * dev->curr_width * formats[dev->curr_fmt_index].pix_size;

        // 确保缓冲区数量，此时 `num_buffers` 会介于 [3, FIFO_SIZE] 之间
        *num_buffers = *num_buffers < 3 ? 3 : *num_buffers;
        *num_buffers = *num_buffers > FIFO_SIZE ? FIFO_SIZE : *num_buffers;

        v4l2_info(&dev->v4l2_dev,
                  "Set num_planes=%d, size=%d, num_buffers=%d\n", *num_planes, sizes[0], *num_buffers);
    } else {
        // 后续调用，校验参数是否正确
        if(*num_planes != 1)
            return -EINVAL;

        if(sizes[0] != dev->curr_height * dev->curr_width * formats[dev->curr_fmt_index].pix_size)
            return -EINVAL;
    }
    return 0;
}

/**
 * @brief: 用户态将缓冲区入队后的回调。
 *      用户空间使用 `VIDIOC_QBUF` 后，缓冲区加入队列之后的回调。在此回调中应当在帧处理完毕后使用 `vb2_buffer_done` 通知框架。
 * @param vb:
 * @note: 本结回调未拥有 `struct video_device.lock` ，需要按需加锁。
 */
static void buf_queue(struct vb2_buffer *vb)
{
    struct vloop_dev *dev = vb->vb2_queue->drv_priv;

    if(vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        kfifo_put(&dev->cap_fifo, vb);

        // 当驱动持有输出缓冲区时，执行调度
        if(!kfifo_is_empty(&dev->out_fifo))
        {
            queue_delayed_work(dev->workqueue, &dev->work, 0);
        }
    } else if(vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
    {
        kfifo_put(&dev->out_fifo, vb);

        // 当驱动持有输入缓冲区时，执行调度
        if(!kfifo_is_empty(&dev->cap_fifo))
        {
            queue_delayed_work(dev->workqueue, &dev->work, 0);
        }
    }
}


/**
 * @brief: 启动流传输回调。
 *      在本驱动中应当完成：
 * @param q:
 * @param count: 执行该回调时已经入队的缓冲区数量
 * @return:
 *      1. 0表示可启动流传输，否则返回负的错误码
 *      2. 当返回错误码时，应当使用 `vb2_buffer_done(vb, VB2_BUF_STATE_QUEUED)` 来归还所有已被用户态预入队的缓冲区，
 *         且状态应当被标记为 `VB2_BUF_STATE_QUEUED`
 *      3. 不过应当尽量避免在 `start_streaming` 中返回错误的情况，应当移动到更早的环节中
 * @note: 本结回调未拥有 `struct video_device.lock` ，需要按需加锁。
 */
static int start_streaming(struct vb2_queue *q, unsigned int count)
{
    struct vloop_dev *dev = q->drv_priv;

    // 需要立即调度一次复制工作，否则在使用非阻塞IO时会触发空读现象：
    //      当用户态首次对capture出队，会有如下的执行逻辑：
    //          用户态capture预入队 -> 用户态调用 `STREAMON` -> device_work -> memcpy -> vb2_buffer_done -> 重新回到队列
    //      但是当用户态使用的是非阻塞IO时，从 `STREAMON` 到 `vb2_buffer_done` 之间需要一段时间，用户态的 `DQBUF` 会立即返回 `-EAGAIN`
    // 从而导致其最开始的一段时间通过非阻塞IO出队结果均为 `-EAGAIN` (OpenCV中就使用的是该方法，可以很简单的复现该情况)
    if(!kfifo_is_empty(&dev->cap_fifo) && !kfifo_is_empty(&dev->out_fifo))
        queue_delayed_work(dev->workqueue, &dev->work, 0);

    return 0;
}


/**
 * @brief: 停止流传输回调
 * @param vq: 所停止的vb2队列
 * @note:
 *      - v4l2不会自动为 `struct video_device.lock` 上锁，但是该回调通常被 `vb2_queue_release` 触发，而此时有锁。
 *      - 并且该回调并不涉及 `struct video_device` 本身的操作
 */
static void stop_streaming(struct vb2_queue *q)
{
    struct vloop_dev *dev = q->drv_priv;

    // 清空FIFO中的缓冲区
    if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
        struct vb2_buffer *vb;
        while (kfifo_get(&dev->cap_fifo, &vb)) {
            vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
        }
    } else if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
        struct vb2_buffer *vb;
        while (kfifo_get(&dev->out_fifo, &vb)) {
            vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
        }
    }
}

/**
 * @brief: videobuf2的操作回调
 * @note: 本结构体中所有回调均未拥有 `struct video_device.lock` ，需要按需加锁!!!
 */
static const struct vb2_ops vfd_qpos = {
    .queue_setup     = queue_setup,           /** 队列配置回调，本驱动将通过该回调敲定输入输出参数 */
    .buf_queue       = buf_queue,             /** 用户态将缓冲区入队后的回调 */
    .start_streaming = start_streaming,       /** 启动流传输回调 */
    .stop_streaming  = stop_streaming,        /** 停止流传输回调 */
};


/**
 * @brief: video_device打开回调，需要在此创建设备上下文
 * @param file: 文件指针
 * @return: 0 if success
 * @note: 本结回调未拥有 `struct video_device.lock` ，需要按需加锁。
 */
static int vloop_open(struct file *file)
{
    int ret = 0;
    // 从文件指针获取设备指针
    struct vloop_dev *dev = video_drvdata(file);
    struct vloop_ctx *ctx = NULL;

    // v4l2_info(&dev->v4l2_dev, "vloop open\n");

    // 1. 分配上下文实例
    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if(!ctx) {
        ret = -ENOMEM;
        goto err_alloc_ctx;
    }

    // 2. 初始化文件句柄
    v4l2_fh_init(&ctx->fh, video_devdata(file));

    // 将驱动私有数据寄存到上下文中
    ctx->dev = dev;

    // 将上下文的文件句柄存入private_data
    // 注意不是存自己定义的上下文实例，因为V4L2内部实现是依赖于其文件句柄
    file->private_data = &ctx->fh;

    // 3. 添加文件句柄到设备列表
    v4l2_fh_add(&ctx->fh);

err_alloc_ctx:
    return ret;
}

/**
 * @brief: 每个用户态打开的引用计数器归0时的回调(而非驱动的最后一个用户态打开的close回调)
 * @note: 需要在该函数中调用 `vb2_queue_release` 释放缓冲区并停止流传输，尽管队列中的缓冲区并不是由 `vloop_open` 分配的
 * @param file: 文件指针
 * @return: 0 if success
 * @note: 本结回调未拥有 `struct video_device.lock` ，需要加锁!!!
 */
static int vloop_release(struct file *file)
{
    struct vloop_ctx *ctx = file2ctx(file);
    struct vloop_dev *dev = ctx->dev;

    // v4l2_info(&dev->v4l2_dev, "vloop release\n");

    // 为struct video_device.lock加锁
    mutex_lock(&dev->dev_mutex);

    // 释放缓冲区并停止流传输，尽管队列中的缓冲区并不是由 `vloop_open` 分配的
    if (ctx->curr_queue == &dev->out_queue)
    {
        // 撤销输出运行标志位
        if (atomic_read(&dev->output_running)) {
            atomic_dec(&dev->output_running);
        }

        // 当output关闭时需要释放两个缓冲区，因为capture的参数取决于output，而驱动无法保证下一次output的参数和当前一致
        vb2_queue_release(&dev->out_queue);
        vb2_queue_release(&dev->cap_queue);
    } else {
        // 而当capture关闭时，理论上不需要关闭output队列
        // 当且仅当最后一个capture被关闭时，释放队列
        vb2_queue_release(&dev->cap_queue);
    }

    // 倒序释放资源
    // 3. 从设备列表删除文件句柄
    v4l2_fh_del(&ctx->fh);
    // 2. 释放文件句柄
    v4l2_fh_exit(&ctx->fh);
    // 1. 释放上下文实例
    kfree(ctx);

    // 解锁
    mutex_unlock(&dev->dev_mutex);

    return 0;
}


/**
 * @brief: mmap
 * @param file
 * @param vma
 * @return
 * @note: 本结回调未拥有 `struct video_device.lock` ，但是该回调通常不需要加锁。
 */
static int vloop_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct vloop_ctx *ctx = file2ctx(file);

    // 根据ctx确定当前打开方向
    if(ctx->curr_queue != NULL)
        return vb2_mmap(ctx->curr_queue, vma);
    else
        return -EINVAL;
}


static __poll_t vloop_poll(struct file *file, poll_table *wait)
{
    struct vloop_ctx *ctx = file2ctx(file);

    // 如果没有设置当前队列，返回错误
    if (!ctx->curr_queue)
        return EPOLLERR;

    return vb2_poll(ctx->curr_queue, file, wait);
}


/**
 * @brief: video_device的文件回调函数
 * @note: 本结构体中所有回调均未拥有 `struct video_device.lock` ，需要按需加锁!!!
 */
static const struct v4l2_file_operations vloop_fops = {
    .owner          = THIS_MODULE,
    .open           = vloop_open,                        /** 设备打开回调函数 */
    .release        = vloop_release,                     /** 设备释放回调函数 */
    .unlocked_ioctl = video_ioctl2,
    .mmap           = vloop_mmap,                        /** 需要自行实现，否则内核不知道要使用哪个队列 */
    .poll           = vloop_poll,                        /** 需要自行实现 */
};

/**
 * @brief: 查询设备能力回调(ioctl(VIDIOC_QUERYCAP, ...))
 */
static int vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
    strscpy(cap->driver, VLOOP_NAME, sizeof(cap->driver));
    strscpy(cap->card, VLOOP_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info),
             "platfrom:%s", VLOOP_NAME);
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
    struct vloop_ctx *ctx = fh;
    struct vloop_dev *dev = ctx->dev;

    // 对于capture设备，其只支持与output相同的类型
    if(atomic_read(&dev->output_running) && f->index == 0)
    {
        f->pixelformat = formats[dev->curr_fmt_index].pix_format;
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
    struct vloop_ctx *ctx   = fh;
    struct vloop_dev *dev   = ctx->dev;

    f->fmt.pix.width        = dev->curr_width;
    f->fmt.pix.height       = dev->curr_height;
    f->fmt.pix.field        = V4L2_FIELD_NONE;                          /** 使用普通逐行扫描，大多数CMOS均为此格式 */
    f->fmt.pix.pixelformat  = formats[dev->curr_fmt_index].pix_format;
    f->fmt.pix.bytesperline = dev->curr_width * formats[dev->curr_fmt_index].pix_size;
    f->fmt.pix.sizeimage    = f->fmt.pix.bytesperline * dev->curr_height;

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
    struct vloop_ctx* ctx = fh;
    struct vloop_dev *dev = ctx->dev;

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
    dev->curr_fmt_index = index;
    dev->curr_width     = f->fmt.pix.width;
    dev->curr_height    = f->fmt.pix.height;

    // 输出目标格式到日志
    v4l2_info(&dev->v4l2_dev, "Set video output format to %c.%c.%c.%c, %dx%d\n",
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
 * @return
 */
static int vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
    struct vloop_ctx* ctx = fh;
    struct vloop_dev *dev = ctx->dev;

    // 验证请求的像素格式在支持列表内
    if(find_format_index(fsize->pixel_format) < 0)
        return -EINVAL;

    // 仅使用一个分辨率
    if (fsize->index != 0)
        return -EINVAL;

    // 填充分辨率
    if(atomic_read(&dev->output_running))
    {
        // 当output方向已敲定数据格式时，仅返回当前使用的格式
        fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
        fsize->discrete.width  = dev->curr_width;
        fsize->discrete.height = dev->curr_height;
    } else {
        // 当output方向未敲定数据格式时，给定驱动配置的分辨率范围
        fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
        fsize->stepwise.min_width   = 1;
        fsize->stepwise.max_width   = MAX_WIDTH;
        fsize->stepwise.step_width  = 1;
        fsize->stepwise.min_height  = 1;
        fsize->stepwise.max_height  = MAX_HEIGHT;
        fsize->stepwise.step_height = 1;
    }

    return 0;
}


/**
 * @brief:
 * @note: 该回调中必须设置当前打开所选定的数据方向，否则部分功能会由于无法识别数据方向导致无法实现
 * @param file
 * @param priv
 * @param rb
 * @return
 */
static int vidioc_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *rb)
{
    struct vloop_ctx *ctx = file2ctx(file);
    struct vloop_dev *dev = ctx->dev;

    // 当释放缓冲区时，重置当前的打开方向
    if(rb->count == 0)
        ctx->curr_queue = NULL;

    // 检查申请的缓冲区类型
    if (rb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE && ctx->curr_queue != &dev->out_queue)
    {
        ctx->curr_queue = &dev->cap_queue;
    } else if (rb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT && ctx->curr_queue != &dev->cap_queue) {
        // 只能有一个输出方向打开
        if(atomic_read(&dev->output_running) != 0)
            return -EBUSY;
        ctx->curr_queue = &dev->out_queue;
    } else
    {
        return -EINVAL;
    }

    return vb2_reqbufs(ctx->curr_queue, rb);
}


/**
 * @brief: 查询已分配的缓冲区信息
 * @param file: 文件句柄，也可从file->private_data中获取fh
 * @param fh: v4l2文件句柄
 * @param b: 要查询的缓冲区实例
 * @return: 0 if success.
 */
static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
    struct vloop_ctx *ctx = fh;

    // 根据不同的缓冲类型找到所属队列
    struct vb2_queue* q = NULL;
    if(b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
        q = &ctx->dev->cap_queue;
    else if(b->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
        q = &ctx->dev->out_queue;
    else
        return -EINVAL;

    return vb2_querybuf(q, b);
}

static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
    struct vloop_ctx *ctx = fh;

    if (ctx->curr_queue == NULL)
        return -EINVAL;

    if (vb2_queue_is_busy(ctx->curr_queue, file))
        return -EBUSY;

    return vb2_qbuf(ctx->curr_queue, ctx->dev->vfd.v4l2_dev->mdev, b);
}

static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
    struct vloop_ctx *ctx = fh;

    if (ctx->curr_queue == NULL)
        return -EINVAL;

    if (vb2_queue_is_busy(ctx->curr_queue, file))
        return -EBUSY;

    return vb2_dqbuf(ctx->curr_queue, b, file->f_flags & O_NONBLOCK);
}

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
    struct vloop_ctx *ctx = fh;

    if (vb2_queue_is_busy(ctx->curr_queue, file))
        return -EBUSY;

    return vb2_streamon(ctx->curr_queue, i);
}

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
    struct vloop_ctx *ctx = fh;

    if (vb2_queue_is_busy(ctx->curr_queue, file))
        return -EBUSY;

    return vb2_streamoff(ctx->curr_queue, i);
}

/**
 * @brief: ioctl操作函数表
 * @note: 本结构体中所有回调均已拥有 `struct video_device.lock` ，不需要额外加锁!!!
 */
static const struct v4l2_ioctl_ops vloop_ioctl_ops = {
    .vidioc_querycap         = vidioc_querycap,          /** 查询设备能力 */
    .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,  /** 枚举video_capture支持的格式(ioctl(VIDIOC_ENUM_FMT)) */
    .vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,  /** 枚举video_output支持的格式(ioctl(VIDIOC_ENUM_FMT)) */
    .vidioc_g_fmt_vid_cap    = vidioc_g_fmt_vid_cap_out, /** 查询capture和output的当前格式 */
    .vidioc_g_fmt_vid_out    = vidioc_g_fmt_vid_cap_out,
    .vidioc_s_fmt_vid_cap    = vidioc_s_fmt_vid_cap,     /** 设置video_capture的数据格式 */
    .vidioc_s_fmt_vid_out    = vidioc_s_fmt_vid_out,     /** 设置video_output的数据格式 */
    .vidioc_enum_framesizes  = vidioc_enum_framesizes,   /** 枚举分辨率 */

    .vidioc_reqbufs		     = vidioc_reqbufs,           /** 需要自行实现，vb2_ioctl_reqbufs无法同时分配输入和输出缓冲区 */
    .vidioc_querybuf	     = vidioc_querybuf,          /** 需要自行实现 */
    .vidioc_qbuf		     = vidioc_qbuf,              /** 需要自行实现 */
    .vidioc_dqbuf		     = vidioc_dqbuf,             /** 需要自行实现 */
    .vidioc_streamon         = vidioc_streamon,          /** 需要自行实现 */
    .vidioc_streamoff        = vidioc_streamoff,         /** 需要自行实现 */
};


/**
 * @brief: video设备被注销时的资源释放回调
 * @note:
 *      触发流程：vloop_exit -> platform_driver_unregister -> vloop_remove -> video_unregister_device -> vloop_device_release
 *      资源管理原则：谁注册谁释放，video_device并没有注册资源，所以不需要释放资源
 * @param vdev
 */
static void vloop_video_release(struct video_device *vdev)
{
    //struct mm2m_dev *dev = container_of(vdev, struct mm2m_dev, vfd);
}


/**
 * @brief: video_device的基本配置
 */
static const struct video_device vloop_videodev =
{
    .name        = VLOOP_NAME,                               /** 暴露于用户空间的设备名称 */
    .vfl_dir     = VFL_DIR_M2M,                              /** 设备数据流向指定为内存到内存，尽管不使用m2m框架 */
    .fops        = &vloop_fops,
    .ioctl_ops   = &vloop_ioctl_ops,
    .release     = vloop_video_release,
    .device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING,
};


/**
 * @brief: 平台设备的探测回调
 * @note: 在本函数中需要对平台设备进行初始化，即初始化vloop设备
 * @param pdev: 需要探测的平台设备
 * @return: 0 if success
 */
static int vloop_probe(struct platform_device *pdev)
{
    int ret = 0;

    // 1. 为设备对象分配内存
    struct vloop_dev *dev = kzalloc(sizeof(struct vloop_dev), GFP_KERNEL);
    if(dev == NULL)
    {
        ret = -ENOMEM;
        goto err_alloc_vloop;
    }
    // 1.2 向平台设备内寄存驱动私有数据
    platform_set_drvdata(pdev, dev);
    // 1.3 初始化输入输出缓冲区队列
    INIT_KFIFO(dev->cap_fifo);
    INIT_KFIFO(dev->out_fifo);
    // 1.4 初始化互斥锁
    mutex_init(&dev->dev_mutex);

    // 2. 注册v4l2设备
    ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
    if(ret != 0)
    {
        goto err_v4l2_register;
    }

    // 3. 注册video_device
    // 3.1 配置vfd成员
    dev->vfd = vloop_videodev;
    //dev->vfd.lock = &dev->dev_mutex;
    dev->vfd.v4l2_dev = &dev->v4l2_dev;
    // 3.2 向video_device中寄存驱动私有数据
    video_set_drvdata(&dev->vfd, dev);
    // 3.3 注册video设备
    ret = video_register_device(&dev->vfd, VFL_TYPE_VIDEO, 0);
    if(ret) {
        goto err_video_register;
    }

    // 4. 初始化vb2队列
    // 4.1 初始化输入队列
    dev->cap_queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    dev->cap_queue.io_modes = VB2_MMAP | VB2_USERPTR;
    dev->cap_queue.drv_priv = dev;
    dev->cap_queue.ops = &vfd_qpos;
    dev->cap_queue.mem_ops = &vb2_vmalloc_memops;
    dev->cap_queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    ret = vb2_queue_init(&dev->cap_queue);
    if (ret) {
        goto err_cap_queue_init;
    }
    // 4.2 初始化输出队列
    dev->out_queue.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    dev->out_queue.io_modes = VB2_MMAP | VB2_USERPTR;
    dev->out_queue.drv_priv = dev;
    dev->out_queue.ops = &vfd_qpos;
    dev->out_queue.mem_ops = &vb2_vmalloc_memops;
    dev->out_queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    ret = vb2_queue_init(&dev->out_queue);
    if (ret) {
        goto err_queue_init;
    }

    // 5. 初始化延迟工作队列
    dev->workqueue = create_singlethread_workqueue("vloop workqueue");
    if(dev->workqueue == NULL)
    {
        goto err_queue_init;
    }

    // 初始化延迟工作对象，并设置回调函数
    INIT_DEFERRABLE_WORK(&dev->work, device_work);

    return ret;

// next err:
    // 5. 释放延迟工作队列
    // destroy_workqueue(dev->workqueue);

err_queue_init:
    // 4. 注销输出队列
    // 注意:
    //  - 不需要在此处手动调用 `vb2_queue_release` 来释放队列，并且 `vb2_queue_release` 不用于释放队列!!!
    //  - `vb2_queue_release` 用于停止流传输并释放缓冲区!!!
    //  - vb2_queue_init只是初始化了**静态**结构，不需要释放
    // vb2_queue_release(&dev->out_queue);  // 不需要!!!
    // vb2_queue_release(&dev->cap_queue);  // 不需要!!!

err_cap_queue_init:
    // 3. 注销video_device
    video_unregister_device(&dev->vfd);

err_video_register:
    // 2. 注销v4l2容器
    v4l2_device_unregister(&dev->v4l2_dev);

err_v4l2_register:
    // 1. 释放设备对象
    kfree(dev);

err_alloc_vloop:
    return ret;
}

/**
 * @brief: 平台设备的设备移除函数
 * @note:
 *      触发流程： vloop_exit -> platform_driver_unregister -> vloop_remove
 *      资源管理原则：谁注册谁释放，因此需要依次释放：
 *      1. workqueue
 *      2. video_device
 *      3. v4l2顶层容器
 *      4. mm2m_dev对象
 * @param pdev
 */
static void vloop_remove(struct platform_device *pdev)
{
    // 获取驱动私有数据
    struct vloop_dev *dev = platform_get_drvdata(pdev);

    // 取消所有任务
    cancel_delayed_work_sync(&dev->work);

    // 5. 释放独有工作队列
    destroy_workqueue(dev->workqueue);
    // 4. 注销输出队列
    // 注意:
    //  - 不需要在此处手动调用 `vb2_queue_release` 来释放队列，并且 `vb2_queue_release` 不用于释放队列!!!
    //  - `vb2_queue_release` 用于停止流传输并释放缓冲区!!!
    //  - vb2_queue_init只是初始化了**静态**结构，不需要释放
    // vb2_queue_release(&dev->out_queue);
    // 4.1 注销输入队列
    // vb2_queue_release(&dev->cap_queue);
    // 3. 释放video_device
    video_unregister_device(&dev->vfd);
    // 2. v4l2顶层容器
    v4l2_device_unregister(&dev->v4l2_dev);
    // 1. 释放mm2m_dev对象
    kfree(dev);
}


/**
 * @brief: 静态管理的平台驱动。驱动在内核中只能是单例模式。
 */
static struct platform_driver vloop_pdrv =
{
    .probe      = vloop_probe,
    .remove     = vloop_remove,
    .driver     = {
        .name   = VLOOP_NAME,             /* 驱动和设备之间通过相同名称进行匹配 */
    },
};


/**
 * @brief: 平台设备引用计数器归零时的释放回调函数
 * @note: 在本回调中只需要释放仅与设备相关的资源，即不需要释放任何资源
 * @param dev: 需要释放的设备
 */
static void vloop_pdev_release(struct device *dev)
{
    pr_info("vloop platform_device released.\n");
}

/**
 * @brief: 静态管理的平台设备，对于虚拟设备通常都使用静态注册的单例模式。
 * @note: 因为是虚拟设备，所以若需要改为多m2m的多例模式，也通常是只使用一个平台设备，然后在probe时将m2m多例化
 */
static struct platform_device vloop_pdev =
{
    .name        = VLOOP_NAME,                /** 驱动和设备之间通过相同名称进行匹配 */
    .dev.release = vloop_pdev_release,         /** 平台设备引用计数器归零时的回调函数 */
};


/**
 * @brief: 模块卸载时的退出函数
 */
static void __exit vloop_exit(void)
{
    platform_driver_unregister(&vloop_pdrv);
    platform_device_unregister(&vloop_pdev);
}

/**
 * @brief: 模块注册时的初始化函数
 * @note: 在该函数中主要负责注册平台设备及平台驱动，具体的vloop功能会在probe函数中初始化
 */
static int __init vloop_init(void)
{
    int ret = 0;

    ret = platform_device_register(&vloop_pdev);
    if (ret)
        return ret;

    ret = platform_driver_register(&vloop_pdrv);
    if (ret)
        platform_device_unregister(&vloop_pdev);

    return ret;
}

MODULE_LICENSE("GPL v2");
module_init(vloop_init);
module_exit(vloop_exit);
