#include <linux/module.h>
#include <linux/device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>

#define MVIDEO_NAME "mvideo"


/**
 * @brief: 设备对象
 */
struct mvideo_dev {
    struct device dev;           /** V4L2所依赖的设备结构, 在实际开发中应直接使用总线模型 **/
    struct v4l2_device v4l2_dev; /** V4L2顶级设备容器 **/
    struct video_device vfd;     /** video设备 **/
    struct vb2_queue queue;      /** vb2缓冲区队列 **/
    uint8_t fill_color[3];       /** 填充颜色 **/
};

// 设备对象指针，配合总线实现时应当定义于probe函数
static struct mvideo_dev *dev;
// video设备指针，配合总线实现时应当定义于probe函数
static struct video_device *vfd;

/**
 * @brief: 最小视频设备的上下文实例
 */
struct mvideo_ctx {
    struct v4l2_fh fh;  /** V4L2的通用文件管理句柄，用于V4L2内部管理用户态的上下文实例 **/
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
        goto error_alloc_ctx;
    }

    // 初始化文件句柄
    v4l2_fh_init(&ctx->fh, video_devdata(file));

    // 将上下文的文件句柄存入private_data
    // 注意不是存自己定义的上下文实例，因为V4L2内部实现是依赖于其文件句柄
    file->private_data = &ctx->fh;

    // 注册文件句柄
    v4l2_fh_add(&ctx->fh);

error_alloc_ctx:
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
    .owner          = THIS_MODULE,              /** 所有者指向本模块 **/
    .open           = mvideo_open,              /** 设备打开回调 **/
    .release        = mvideo_release,           /** 设备释放回调 **/
    .unlocked_ioctl = video_ioctl2,
    .mmap           = v4l2_m2m_fop_mmap,
};


static int queue_setup(struct vb2_queue *vq, unsigned int *num_buffers, unsigned int *num_planes,
                                unsigned int sizes[], struct device *alloc_devs[])
{
    struct virt_cam_device *cam = vb2_get_drv_priv(vq);

    // 单平面RGB888
    *num_planes = 1;
    sizes[0] = cam->width * cam->height * 3; // RGB888 = 3字节/像素

    return 0;
}

/**
 * @brief: videobuf2的操作回调
 */
static const struct vb2_ops mvideo_qops = {
    .queue_setup     = queue_setup,
    .buf_prepare     = buffer_prepare,
    .buf_queue       = buffer_queue,
    .start_streaming = start_streaming,
    .stop_streaming  = stop_streaming,
};


/**
 * @brief: 查询设备能力回调(ioctl(VIDIOC_QUERYCAP, ...))
 */
static int vidioc_querycap(struct file *file, void *priv,
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
 * @param priv:
 * @param f:
 * @return
 */
static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
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
 * @param file: 用户空间操作的文件句柄，正常驱动应当从中获取上下文实例
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    f->fmt.pix.width        = 640;                 /** 分辨率选定为640*480 **/
    f->fmt.pix.height       = 480;
    f->fmt.pix.field        = V4L2_FIELD_NONE;     /** 使用普通逐行扫描，大多数CMOS均为此格式 **/
    f->fmt.pix.pixelformat  = V4L2_PIX_FMT_RGB24;  /** 使用RGB 888数据格式 **/
    return 0;
}

/**
 * @brief: 设置video_capture的数据格式
 * @note: 最简驱动中仅允许设置为640*480、RGB888、逐行扫描，且驱动应当强行修改到目标格式(V4L2标准语义要求)。
 * @param file: 用户空间操作的文件句柄，正常驱动应当从中获取上下文实例
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    f->fmt.pix.width        = 640;                 /** 分辨率选定为640*480 **/
    f->fmt.pix.height       = 480;
    f->fmt.pix.field        = V4L2_FIELD_NONE;     /** 使用普通逐行扫描，大多数CMOS均为此格式 **/
    f->fmt.pix.pixelformat  = V4L2_PIX_FMT_RGB24;  /** 使用RGB 888数据格式 **/
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
    .vidioc_querycap         = vidioc_querycap,          /** 查询设备能力 **/
    .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,  /** 枚举video_capture支持的格式(ioctl(VIDIOC_ENUM_FMT)) **/
    .vidioc_g_fmt_vid_cap    = vidioc_g_fmt_vid_cap,     /** 查询video_capture的当前格式 **/
    .vidioc_s_fmt_vid_cap    = vidioc_s_fmt_vid_cap,     /** 设置video_capture的数据格式 **/
    .vidioc_enum_framesizes  = vidioc_enum_framesizes,   /** 枚举分辨率 **/

    .vidioc_reqbufs          = vb2_ioctl_reqbufs,        /** 使用videobuf2提供的机制 **/
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
static void mvideo_video_device_release(struct video_device *vdev)
{
    v4l2_device_unregister(&dev->v4l2_dev);
}

/**
 * @brief: 视频设备模型
 */
static const struct video_device mvideo_videodev = {
    .name        = MVIDEO_NAME,                                   /** 暴露于用户空间的设备名 **/
    .vfl_dir     = VFL_DIR_RX,                                    /** 定义为接收设备(用户态视角接收数据) **/
    .device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE ,  /** 设备能力注册为流设备和捕获设备 **/
    .fops        = &mvideo_fops,                                  /** VFS操作接口 **/
    .ioctl_ops   = &mvideo_ioctl_ops,                             /** ioctl操作函数表 **/
    //.minor       = -1,                                            /**  **/
    .release     = mvideo_video_device_release,                   /** 设备释放接口 **/
};

/**
 * @brief: 基础设备模型释放函数
 * @note: 调用时机为dev的引用计数器变0
 * @param dev
 */
static void mvideo_device_release(struct device *dev)
{
    kfree(container_of(dev, struct mvideo_dev, dev));
    printk(KERN_INFO "mvideo device released.\n");
}

/**
 * @brief: 卸载设备和模块
 */
static void __exit mvideo_exit(void)
{
    video_unregister_device(vfd);
    device_unregister(&dev->dev); // 该步骤会自动
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

    // 初始化基础设备模型
    device_initialize(&dev->dev);

    // 配置基础设备模型
    dev->dev.release = mvideo_device_release;
    dev_set_name(&dev->dev, MVIDEO_NAME);

    // 注册基础设备模型
    ret = device_add(&dev->dev);
    if (ret) {
        goto err_device_add;
    }

    // 当不为基础设备绑定驱动时，必须为v4l2_dev设置name
    strscpy(dev->v4l2_dev.name, MVIDEO_NAME, sizeof(dev->v4l2_dev.name));

    // 注册v4l2_device
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

    // 注册视频设备
    ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
    if(ret) {
        goto err_video_gister;
    }

    // 输出视频设备引索(次设备号、/dev/videoX的序号)
    v4l2_info(&dev->v4l2_dev, "Device registered as /dev/video%d\n", vfd->num);

    // 初始化videobuf2
    dev->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    dev->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
    dev->queue.drv_priv = cam;
    dev->queue.ops = &mvideo_qops;
    dev->queue.mem_ops = &vb2_vmalloc_memops;                        // 指定使用vmalloc内存
    dev->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    ret = vb2_queue_init(&dev->queue);
    if(ret) {
        goto err_vb2_init;
    }


    return ret;

    // next err
    // vb2_queue_release(&dev->queue);

err_vb2_init:
    video_unregister_device(vfd);
    return ret;

err_video_gister:
    v4l2_device_unregister(&dev->v4l2_dev);

err_v4l2_register:
    device_unregister(&dev->dev);

err_device_add:
    put_device(&dev->dev);

err_alloc_dev:
    return ret;
}

MODULE_LICENSE("GPL v2");
module_init(mvideo_init);
module_exit(mvideo_exit);
