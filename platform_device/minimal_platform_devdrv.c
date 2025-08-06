#define MPDEV_NAME        "mpdev"

#ifndef pr_fmt
#define pr_fmt(fmt)       MPDEV_NAME ": " fmt
#endif

#include <linux/platform_device.h>
#include <linux/printk.h>


/**********************************************************************************************************************/
/*                                              下方为设备侧代码，仅与设备相关                                              */
/**********************************************************************************************************************/

/**
 * @brief: 当设备的引用计数器归零时触发该回调
 * @note:
 *      1. 该函数应当释放仅与设备相关的资源，即设备在还没被驱动绑定时必须的资源(设备可以在无驱动时存在)，即仅包含设备侧的资源
 * @param dev:
 */
static void mpdev_release(struct device *dev)
{
    pr_info("mpdev device released.\n");
}

/**
 * @brief: 静态管理的单例平台设备
 * @note: 对于虚拟设备，往往都是静态单例模式
 */
static struct platform_device mpdev = {
    .name        = MPDEV_NAME,                  // 设备名称标识符
    .dev.release = mpdev_release,               // 设备资源释放函数，当设备计数器归0时被调用
};

/**********************************************************************************************************************/
/*                                              下方为驱动侧代码，仅与驱动相关                                              */
/**********************************************************************************************************************/

/**
 * @brief: 平台设备的探测回调
 * @param pdev:
 * @return:
 */
static int mpdev_probe(struct platform_device *pdev)
{
    // 可绑定私有数据
    // platform_set_drvdata(pdev, priv);

    pr_info("mpdev device probed.\n");

    return 0;
}

/**
 * @brief: 设备在移除或卸载时触发
 * @note:
 *      1. 卸载模块时，内核会调用 `mpdev_exit` 从而触发该回调
 *      2. 需要注意，设备可以在无驱动时存在
 *      3. 该回调应当值回收与驱动相关的资源，即仅包含驱动侧的资源，仅与设备相关的资源应当在 `mpdev_release` 中回收
 *      4. 该回调执行完毕后，设备的引用计数器会减一，也因此其必定会在 `mpdev_release` 前被调用
 * @param pdev
 */
static void mpdev_remove(struct platform_device *pdev)
{
    // 可获取绑定的驱动数据
    // void *priv = platform_get_drvdata(pdev);
    // 释放对应的驱动数据
    // kfree(priv);

    pr_info("mpdev device removed.\n");
}

/**
 * @brief: 静态单例模式的平台设备驱动
 */
static struct platform_driver mpdrv = {
    .probe       = mpdev_probe,                 // 设备初始化的入口函数
    .remove_new  = mpdev_remove,                // 设备被移除时触发回调(移除或卸载时触发)
    .driver      = {
        .name    = MPDEV_NAME,                  // 驱动名
    },
};

/**********************************************************************************************************************/
/*                                            下方为公用代码，负责注册设备和驱动                                             */
/**********************************************************************************************************************/

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
