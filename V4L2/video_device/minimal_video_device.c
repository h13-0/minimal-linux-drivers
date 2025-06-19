#include <linux/module.h>
#include <linux/device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>

#define MVIDEO_NAME "mvideo"


/**
 * @brief: 设备对象
 */
struct mvideo_dev {
    struct device dev;           /** V4L2所依赖的设备结构, 在实际开发中应直接使用总线模型 **/
    struct v4l2_device v4l2_dev; /** V4L2顶级设备容器 **/
    struct video_device vfd;     /** video设备 **/
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

static int vidioc_g_fmt(struct mvideo_ctx *ctx, struct v4l2_format *f)
{
    f->fmt.pix.width        = 640;
    f->fmt.pix.height       = 480;
    f->fmt.pix.field        = V4L2_FIELD_NONE;
    f->fmt.pix.pixelformat  = v4l2_fourcc('R', 'G', 'B', '3');
    return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
                                struct v4l2_format *f)
{
    return vidioc_g_fmt(file2ctx(file), f);
}


/**
 * @brief: ioctl操作函数表
 */
static const struct v4l2_ioctl_ops mvideo_ioctl_ops = {
    .vidioc_querycap = vidioc_querycap,           /** 查询设备能力 **/
    .vidioc_g_fmt_vid_cap = vidioc_g_fmt_vid_cap, /** video_capture支持的数据格式 **/
};

/**
 * @brief: video设备的释放接口
 * @param vdev
 */
static void mvideo_video_device_release(struct video_device *vdev)
{
    v4l2_device_unregister(&dev->v4l2_dev);
}

/**
 * @brief: 视频设备模型
 */
static const struct video_device mvideo_videodev = {
    .name        = MVIDEO_NAME,                  /** 暴露于用户空间的设备名 **/
    .vfl_dir     = VFL_DIR_TX,                   /** 定义为发送设备 **/
    .device_caps = V4L2_CAP_STREAMING,           /** 设备能力注册为流设备 **/
    .fops        = &mvideo_fops,                  /** VFS操作接口 **/
    .ioctl_ops   = &mvideo_ioctl_ops,             /** ioctl操作函数表 **/
    //.minor       = -1,                           /**  **/
    .release     = mvideo_video_device_release,         /** 设备释放接口 **/
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
        goto error_alloc_dev;
    }

    // 初始化基础设备模型
    device_initialize(&dev->dev);

    // 配置基础设备模型
    dev->dev.release = mvideo_device_release;
    dev_set_name(&dev->dev, MVIDEO_NAME);

    // 注册基础设备模型
    ret = device_add(&dev->dev);
    if (ret) {
        goto error_device_add;
    }

    // 当不为基础设备绑定驱动时，必须为v4l2_dev设置name
    strscpy(dev->v4l2_dev.name, MVIDEO_NAME, sizeof(dev->v4l2_dev.name));

    // 注册v4l2_device
    ret = v4l2_device_register(&dev->dev, &dev->v4l2_dev);
    if(ret) {
        goto error_v4l2_register;
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
        goto error_video_register;
    }

    // 输出视频设备引索(次设备号、/dev/videoX的序号)
    v4l2_info(&dev->v4l2_dev, "Device registered as /dev/video%d\n", vfd->num);

    return ret;

    // video_unregister_device(vfd);
    // return ret;

error_video_register:
    v4l2_device_unregister(&dev->v4l2_dev);

error_v4l2_register:
    device_unregister(&dev->dev);

error_device_add:
    put_device(&dev->dev);

error_alloc_dev:
    return ret;
}

MODULE_LICENSE("GPL v2");
module_init(mvideo_init);
module_exit(mvideo_exit);
