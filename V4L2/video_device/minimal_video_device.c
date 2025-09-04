#define MVIDEO_NAME "mvideo"

#ifndef pr_fmt
#define pr_fmt(fmt) "mvideo: " fmt
#endif

#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/printk.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>


/**
 * @brief: 设备对象
 */
struct mvideo_dev {
    struct device dev;           /** V4L2所依赖的设备结构, 在实际开发中应直接使用总线模型 */
    struct v4l2_device v4l2_dev; /** V4L2顶级设备容器 */
    struct video_device vfd;     /** video设备 */
    struct vb2_queue queue;      /** vb2缓冲区队列 */
    uint8_t fill_color[3];       /** 填充颜色 */
    uint16_t width;              /** 分辨率 */
    uint16_t height;
};

// 设备对象指针，配合总线实现时应当定义于probe函数
static struct mvideo_dev *dev;
// video设备指针，配合总线实现时应当定义于probe函数
static struct video_device *vfd;

// 填充颜色
static uint8_t color_r = 0;
static uint8_t color_g = 0;
static uint8_t color_b = 0;

/**
 * @brief: 最小视频设备的上下文实例
 */
struct mvideo_ctx {
    struct v4l2_fh fh;  /** V4L2的通用文件管理句柄，用于V4L2内部管理用户态的上下文实例 */
    // 在正常驱动开发中，通常会在本结构体的后续记录当前格式信息等
};

/**
 * @brief: 从文件指针中获取最小视频设备的上下文实例
 * @param file: fops传来的文件指针
 * @return: 上下文实例指针
 */
static inline struct mvideo_ctx *file2ctx(struct file *file)
{
    return container_of(file->private_data, struct mvideo_ctx, fh);
}

/**
 * @brief: 视频设备文件打开接口(/dev/videoX)
 * @param file: 文件指针
 * @return: 0 if success.
 */
static int mvideo_open(struct file *file)
{
    int ret = 0;
    struct mvideo_ctx *ctx = NULL;

    // 分配上下文实例
    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if(!ctx) {
        ret = -ENOMEM;
        goto err_alloc_ctx;
    }

    // 初始化文件句柄
    v4l2_fh_init(&ctx->fh, video_devdata(file));

    // 将上下文的文件句柄存入private_data
    // 注意不是存自己定义的上下文实例，因为V4L2内部实现是依赖于其文件句柄
    file->private_data = &ctx->fh;

    // 注册文件句柄
    v4l2_fh_add(&ctx->fh);

err_alloc_ctx:
    return ret;
}

/**
 * @brief: 设备释放函数，当打开计数器归零时被调用
 * @param file: 文件指针
 * @return: 0 if success.
 */
static int mvideo_release(struct file *file)
{
    struct mvideo_ctx *ctx = file2ctx(file);
    v4l2_fh_del(&ctx->fh);
    v4l2_fh_exit(&ctx->fh);
    kfree(ctx);
    return 0;
}

/**
 * @brief: VFS操作接口
 */
static const struct v4l2_file_operations mvideo_fops = {
    .owner          = THIS_MODULE,              /** 所有者指向本模块 */
    .open           = mvideo_open,              /** 设备打开回调 */
    .release        = mvideo_release,           /** 设备释放回调 */
    .unlocked_ioctl = video_ioctl2,
    .mmap           = vb2_fop_mmap,
};

/**
 * @brief: "模拟"将buffer提交给硬件的函数
 * @param vb: 需要填充数据的buffer
 */
static void submit_buffer(struct vb2_buffer *vb)
{
    // 默认buffer处理结果为ERROR
    enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
    // 向缓存中填充数据，在普通驱动中应当由硬件完成
    // 1. 获取buffer指定平面的虚拟地址
    uint8_t *addr = vb2_plane_vaddr(vb, 0);
    if(addr != NULL)
    {
        // 2. 填充数据
        for(int i = 0; i < dev->height; i++)
        {
            for(int j = 0; j < dev->width; j++)
            {
                *addr++ = dev->fill_color[0]; // 先赋值后移动指针
                *addr++ = dev->fill_color[1];
                *addr++ = dev->fill_color[2];
            }
        }

        // 3. 设置实际填充字节数
        vb2_set_plane_payload(vb, 0, dev->width * dev->height * 3);

        // 4. 设置时间戳
        vb->timestamp = ktime_get_ns();
        state = VB2_BUF_STATE_DONE;
    } else {
        v4l2_err(&dev->v4l2_dev, "get vb2_plane_vaddr failed.\n");
    }

    // 在普通驱动中，应当在硬件处理完成后通过中断等方式触发 `vb2_buffer_done` 。
    vb2_buffer_done(vb, state);
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
    if(*num_planes == 0) {
        // 首次调用，此时驱动进行参数配置
        // 由于仅支持RGB888，因此平面配置可忽略用户配置的像素格式
        // 配置为单平面
        *num_planes = 1;
        // 每个平面的总字节数为 w*h*3
        sizes[0] = dev->width * dev->height * 3;

        // 确保缓冲区数量，此时 `num_buffers` 会介于 [0, VIDEO_MAX_FRAME] 之间
        v4l2_info(&dev->v4l2_dev, "Initial num_buffers is:%d\n", *num_buffers);
        if(*num_buffers < 3) {
            *num_buffers = 3;
        }
    } else {
        // 后续调用，校验参数是否正确
        if(*num_planes != 1 || sizes[0] != dev->width * dev->height * 3) {
            return -EINVAL;
        }
    }
    return 0;
}


/**
 * @brief: 用户态将缓冲区入队后的回调。
 *      用户空间使用 `VIDIOC_QBUF` 后，缓冲区加入队列之后的回调。在此回调中驱动应当启动硬件操作，
 *      并在帧填充完毕后使用 `vb2_buffer_done` 通知框架。
 * @param vb:
 */
static void buffer_queue(struct vb2_buffer *vb)
{
    // 将buffer提交给"硬件"处理
    submit_buffer(vb);
}


/**
 * @brief: 启动流传输回调。
 *      在普通驱动中应当完成：
 *      1. 确保硬件有足够的缓冲区开始工作
 *      2. 初始化硬件并启动数据流
 *      3. 当驱动程序发生错误时，已经被 `buf_queue` 取出的缓冲区需要通过 `vb2_buffer_done` 来标记和归还缓冲区。
 *          需要注意，其应当将缓冲区标记为 `VB2_BUF_STATE_QUEUED` 。
 *      在本驱动中不需要处理任何事情
 * @param q:
 * @param count: 执行该回调时已经入队的缓冲区数量
 * @return
 */
static int start_streaming(struct vb2_queue *q, unsigned int count)
{
    // 在本驱动中不需要处理任何事情
    return 0;
}


/**
 * @brief: 停止流传输回调
 *      在普通驱动中应当完成：
 *      1. 停止所有硬件传输，确保不再访问所有缓冲区
 *      2. 返回已留给驱动的缓冲区到 `DEQUEUED` 状态从而方便内核及用户态安全释放资源
 *          - 具体而言，此时缓冲区可能有如下几种状态：
 *          - `QUEUED` ：已经入队但还未被驱动处理的缓冲区
 *          - `ACTIVE` ：硬件正在处理的缓冲区
 *          - `DONE` ：已经被驱动处理完成但还未被用户态取走的缓冲区
 *          - `DEQUEUED` ：已经被用户态取走的缓冲区
 *          上述若干状态中，需要处理的缓冲区状态为 `ACTIVE` ，其均需要通过 `vb2_buffer_done` 返回到 `DEQUEUED` ，
 *          且需要注意：
 *          - `ACTIVE` 状态必须返回为 `ERROR` 状态，因为实际上该缓冲区并未正确填充，即：
 *              - `vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);`
 *      3. 手动调用 `vb2_ops.buf_finish` 进行后处理(如果实现了该函数的话)
 * @param vq: 所停止的vb2队列
 */
static void stop_streaming(struct vb2_queue *vq)
{
    // 在实际驱动中需要停止硬件操作所有缓冲区之后才可执行后续操作，但是本驱动中缓冲区仅会在buffer_queue中操作，且其不会与本回调同时被执行。
    // 也因此本函数中不会有需要操作的 ACTIVE 缓冲区，直接返回即可。
    // 对于从 ACTIVE 返回的缓冲区，如果有 `vb2_ops.buf_finish` 的后处理需求时应当调用。
}

/**
 * @brief: videobuf2的操作回调
 * @note: 只实现必要的回调
 */
static const struct vb2_ops mvideo_qops = {
    .queue_setup     = queue_setup,           /** 队列配置回调 */
    .buf_queue       = buffer_queue,          /** 用户态将缓冲区入队后的回调 */
    .start_streaming = start_streaming,       /** 启动流传输回调 */
    .stop_streaming  = stop_streaming,        /** 停止流传输回调 */
};


/**
 * @brief: 查询设备能力回调(ioctl(VIDIOC_QUERYCAP, ...))
 */
static int vidioc_querycap(struct file *file, void *fh,
                           struct v4l2_capability *cap)
{
    strscpy(cap->driver, MVIDEO_NAME, sizeof(cap->driver));
    strscpy(cap->card, MVIDEO_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info),
             "NULL:%s", MVIDEO_NAME);
    return 0;
}

/**
 * @brief: 枚举作为video_capture时支持的格式
 * @note: 最简驱动中统一固定为RGB888
 * @param file:
 * @param fh:
 *      通常指向上下文实例的 `struct v4l2_fh` 成员 。实际上指向的是 `mvideo_open` 函数中设置的 `file->private_data` 的值。
 *      但是通常会把 `struct v4l2_fh` 放到上下文实例(`mvideo_ctx`)的第一个成员中，因此通常二者地址相同。
 * @param f:
 * @return
 */
static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
    v4l2_info(&dev->v4l2_dev, "enum videoc fmt cap at index:%d\n", f->index);
    if(f->index == 0)
    {
        f->pixelformat = V4L2_PIX_FMT_RGB24;
        return 0;
    } else {
        return -EINVAL;
    }
}

/**
 * @brief: 查询video_capture的当前格式
 * @note: 最简驱动中统一固定为640x480, RGB888
 * @param file: 用户空间操作的文件句柄
 * @param fh:
 *      通常指向上下文实例的 `struct v4l2_fh` 成员 。实际上指向的是 `mvideo_open` 函数中设置的 `file->private_data` 的值。
 *      但是通常会把 `struct v4l2_fh` 放到上下文实例(`mvideo_ctx`)的第一个成员中，因此通常二者地址相同。
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    f->fmt.pix.width        = 640;                 /** 分辨率选定为640*480 */
    f->fmt.pix.height       = 480;
    f->fmt.pix.field        = V4L2_FIELD_NONE;     /** 使用普通逐行扫描，大多数CMOS均为此格式 */
    f->fmt.pix.pixelformat  = V4L2_PIX_FMT_RGB24;  /** 使用RGB 888数据格式 */
    return 0;
}

/**
 * @brief: 设置video_capture的数据格式
 * @note:
 *      最简驱动中仅允许设置为640*480、RGB888、逐行扫描，且驱动应当强行修改到目标格式(V4L2标准语义要求)。
 *      文档规定：除非 `f->type` 字段错误，否则不应返回错误代码，因此直接修改格式和参数即可。
 * @param file: 用户空间操作的文件句柄
 * @param fh:
 *      通常指向上下文实例的 `struct v4l2_fh` 成员 。实际上指向的是 `mvideo_open` 函数中设置的 `file->private_data` 的值。
 *      但是通常会把 `struct v4l2_fh` 放到上下文实例(`mvideo_ctx`)的第一个成员中，因此通常二者地址相同。
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    f->fmt.pix.width        = 640;                 /** 分辨率选定为640*480 */
    f->fmt.pix.height       = 480;
    f->fmt.pix.field        = V4L2_FIELD_NONE;     /** 使用普通逐行扫描，大多数CMOS均为此格式 */
    f->fmt.pix.pixelformat  = V4L2_PIX_FMT_RGB24;  /** 使用RGB 888数据格式 */
    return 0;
}

/**
 * @brief: 枚举分辨率的回调函数(ioctl(VIDIOC_ENUM_FRAMESIZES))
 * @param file:
 * @param fh:
 *      通常指向上下文实例的 `struct v4l2_fh` 成员 。实际上指向的是 `mvideo_open` 函数中设置的 `file->private_data` 的值。
 *      但是通常会把 `struct v4l2_fh` 放到上下文实例(`mvideo_ctx`)的第一个成员中，因此通常二者地址相同。
 * @param fsize: 返回给用户空间的分辨率信息
 * @return
 */
static int vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
    // 仅支持一个离散分辨率
    if(fsize->index != 0) {
        return -EINVAL;
    }

    // 仅支持RGB888
    if(fsize->pixel_format != V4L2_PIX_FMT_RGB24) {
        return -EINVAL;
    }

    // 设置分辨率类型为离散
    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete.width = 640;
    fsize->discrete.height = 480;
    return 0;
}

/**
 * @brief: ioctl操作函数表
 */
static const struct v4l2_ioctl_ops mvideo_ioctl_ops = {
    .vidioc_querycap         = vidioc_querycap,          /** 查询设备能力 */
    .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,  /** 枚举video_capture支持的格式(ioctl(VIDIOC_ENUM_FMT)) */
    .vidioc_g_fmt_vid_cap    = vidioc_g_fmt_vid_cap,     /** 查询video_capture的当前格式 */
    .vidioc_s_fmt_vid_cap    = vidioc_s_fmt_vid_cap,     /** 设置video_capture的数据格式 */
    .vidioc_enum_framesizes  = vidioc_enum_framesizes,   /** 枚举分辨率 */

    .vidioc_reqbufs          = vb2_ioctl_reqbufs,        /** 使用videobuf2提供的机制 */
    .vidioc_querybuf         = vb2_ioctl_querybuf,
    .vidioc_qbuf             = vb2_ioctl_qbuf,
    .vidioc_dqbuf            = vb2_ioctl_dqbuf,
    .vidioc_streamon         = vb2_ioctl_streamon,
    .vidioc_streamoff        = vb2_ioctl_streamoff,
};

/**
 * @brief: video设备的释放接口
 * @param vdev: 通常用于获取设备实例，在最简驱动中不需要
 */
static void mvideo_device_release(struct video_device *vdev)
{
    vb2_queue_release(&dev->queue);
}

/**
 * @brief: 视频设备模型
 */
static const struct video_device mvideo_videodev = {
    .name        = MVIDEO_NAME,                                   /** 暴露于用户空间的设备名 */
    .vfl_dir     = VFL_DIR_RX,                                    /** 定义为接收设备(用户态视角接收数据) */
    .device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE ,  /** 设备能力注册为流设备和捕获设备 */
    .fops        = &mvideo_fops,                                  /** VFS操作接口 */
    .ioctl_ops   = &mvideo_ioctl_ops,                             /** ioctl操作函数表 */
    //.minor       = -1,                                            /**  */
    .release     = mvideo_device_release,                          /** 设备释放接口 */
};

/**
 * @brief: 基础设备模型释放函数
 * @note: 调用时机为dev的引用计数器变0
 * @param dev
 */
static void device_release(struct device *dev)
{
    kfree(container_of(dev, struct mvideo_dev, dev));
    pr_info("mvideo device released.\n");
}

/**
 * @brief: 卸载设备和模块
 */
static void __exit mvideo_exit(void)
{
    if (!dev) return;

    if (vfd) {
        video_unregister_device(vfd);
        vfd = NULL;
    }

    v4l2_device_unregister(&dev->v4l2_dev);
    // device_unregister会减少两次计数器，因此不再需要手动put_device
    device_unregister(&dev->dev);
    dev = NULL;
}

/**
 * @brief: 注册模块及设备
 * @note: 最小驱动中不使用Linux总线模型，在实际使用中可以考虑使用platfrom总线或实际物理总线。
 * @return: 0 if success.
 */
static int __init mvideo_init(void)
{
    int ret;

    // 分配设备对象的内存
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if(!dev) {
        ret = -ENOMEM;
        goto err_alloc_dev;
    }
    // 填充基础参数
    dev->fill_color[0] = color_r;
    dev->fill_color[1] = color_g;
    dev->fill_color[2] = color_b;
    dev->width = 640;
    dev->height = 480;

    // 初始化基础设备模型，并将dev->dev的引用计数器置为1(引用计数器为dev->dev.kobj.kref)
    device_initialize(&dev->dev);

    // 配置基础设备模型
    dev->dev.release = device_release;
    dev_set_name(&dev->dev, MVIDEO_NAME);

    // 注册基础设备模型，随后dev->dev的引用计数器为2
    ret = device_add(&dev->dev);
    if (ret) {
        goto err_device_add;
    }

    // 当不为基础设备绑定驱动时，必须为v4l2_dev设置name
    strscpy(dev->v4l2_dev.name, MVIDEO_NAME, sizeof(dev->v4l2_dev.name));

    // 注册v4l2_device，随后dev->dev的引用计数器为3
    ret = v4l2_device_register(&dev->dev, &dev->v4l2_dev);
    if(ret) {
        goto err_v4l2_register;
    }

    // 配置视频设备模型
    dev->vfd = mvideo_videodev;
    vfd = &dev->vfd;
    vfd->v4l2_dev = &dev->v4l2_dev;

    // 向video_device中寄存私有数据
    video_set_drvdata(vfd, dev);

    // 注册视频设备，随后dev->dev的引用计数器为4
    ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
    if(ret) {
        goto err_video_register;
    }

    // 输出视频设备引索(次设备号、/dev/videoX的序号)
    v4l2_info(&dev->v4l2_dev, "Device registered as /dev/video%d\n", vfd->num);

    // 初始化videobuf2
    dev->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;                   // 指定为视频捕获设备
    dev->queue.io_modes = VB2_MMAP | VB2_USERPTR;
    dev->queue.drv_priv = dev;
    dev->queue.ops = &mvideo_qops;
    dev->queue.mem_ops = &vb2_vmalloc_memops;                        // 指定使用vmalloc内存
    dev->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    ret = vb2_queue_init(&dev->queue);
    if(ret) {
        goto err_vb2_init;
    }
    dev->vfd.queue = &dev->queue;

    return ret;

    // next err
    // vb2_queue_release(&dev->queue);

err_vb2_init:
    video_unregister_device(vfd);

err_video_register:
    v4l2_device_unregister(&dev->v4l2_dev);

err_v4l2_register:
    // 此处不可使用device_unregister，因为其会使计数器-2
    device_del(&dev->dev);

err_device_add:
    // 减少引用计数器，并触发 `device_release`
    put_device(&dev->dev);
    dev = NULL;            // 清空静态变量指针
    vfd = NULL;

err_alloc_dev:
    return ret;
}

module_param(color_r, byte, S_IRUGO);
module_param(color_g, byte, S_IRUGO);
module_param(color_b, byte, S_IRUGO);

MODULE_LICENSE("GPL v2");
module_init(mvideo_init);
module_exit(mvideo_exit);
