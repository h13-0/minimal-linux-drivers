#define MPDEV_NAME        "mpdev"

#ifndef pr_fmt
#define pr_fmt(fmt) MPDEV_NAME ": " fmt
#endif

#include <linux/platform_device.h>
#include <linux/printk.h>

static void mpdev_release(struct device *dev)
{
    pr_info("mpdev device released.\n");
}

static struct platform_device mpdev = {
    .name        = MPDEV_NAME,                  // 设备名称标识符
    .dev.release = mpdev_release,               // 设备资源释放函数，当设备计数器归0时被调用
};

static int mpdev_probe(struct platform_device *pdev)
{
    // 可绑定私有数据
    // platform_set_drvdata(pdev, priv);

    pr_info("mpdev device probed.\n");

    return 0;
}

static void mpdev_remove(struct platform_device *pdev)
{
    pr_info("mpdev device removed.\n");
}

static struct platform_driver mpdrv = {
    .probe       = mpdev_probe,                 // 设备初始化的入口函数
    .remove_new  = mpdev_remove,                // 设备被移除时触发回调(移除或卸载时触发)
    .driver      = {
        .name    = MPDEV_NAME,                  // 驱动名
    },
};

static void __exit mpdev_exit(void)
{
    platform_driver_unregister(&mpdrv);
    platform_device_unregister(&mpdev);
}

static int __init mpdev_init(void)
{
    int ret;

    ret = platform_device_register(&mpdev);
    if (ret)
        return ret;

    ret = platform_driver_register(&mpdrv);
    if (ret)
        platform_device_unregister(&mpdev);

    return ret;
}

MODULE_LICENSE("GPL v2");
module_init(mpdev_init);
module_exit(mpdev_exit);
