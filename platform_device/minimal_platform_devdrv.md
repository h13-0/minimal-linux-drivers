---
number headings: auto, first-level 1, max 6, 1.1
---
#嵌入式 #Linux驱动开发 #V4L2 

# 1 最小平台设备示例

# 2 目录

```toc
```

# 3 特性与基本实现

## 3.1 基本特性

该示例拥有如下的基本特性：
- 驱动特性：
	1. 该设备在 `sysfs` 中的路径为 `/sys/devices/platform/mpdev.x` 
	2. 该驱动在 `sysfs` 中的路径为 `/sys/bus/platform/drivers/mpdev`
	3. 使用设备和驱动名称进行驱动匹配

## 3.2 注意点

注意：
1. `minimal_platform_devdrv.c` 将平台设备和平台驱动编写到了同一个源文件中，但是<font color="#c00000">应当注意区分</font><span style="background:#fff88f"><font color="#c00000">哪些代码是设备的功能</font></span>、<span style="background:#fff88f"><font color="#c00000">哪些代码是驱动的功能</font></span>。
	1. 尤其是注意区分 `mpdev_release` 和 `mpdev_remove` 两个函数的功能。
	2. 在该文件中已经通过注释将代码分为 `设备侧代码` 、 `驱动侧代码` 和 `驱动侧代码` 三部分。
