---
number headings: auto, first-level 1, max 6, 1.1
---
#嵌入式 #Linux驱动开发 #V4L2 

# 1 最小video_device示例


本示例依赖系统模块 `videobuf2_v4l2` 和 `videobuf2_vmalloc`
# 2 目录

```toc
```

# 3 简介、特性与实现

简介：
	`mvideo`<font color="#c00000"> 是一个<del>最简</del>的V4L2视频采集内核模块示例</font>。驱动在用户态请求缓冲区后，由内核端直接把固定分辨率(640×480)、固定像素格式(RGB888，`V4L2_PIX_FMT_RGB24`)的数据填充成纯色帧，通过 `videobuf2` (VB2)框架回送给用户态。
目的：
- 演示最简的虚拟摄像头实现

## 3.1 基本特性

该示例拥有如下特性：
- 驱动特性：
	1. 使用最简device结构而不使用平台设备等总线模型
	2. 基础总线模型的挂载位置位于 `/sys/devices/mvideo`
	3. `v4l2_device` 的挂载位置位于 `/sys/devices/mvideo/v4l2-device`
	4. `video` 设备的文件位于 `/dev/videoX`
- V4L2特性：
	1. 设备能力设置为 `V4L2_CAP_STREAMING`
	2. 支持枚举输出格式( `ioctl(VIDIOC_ENUM_FMT)` )
	3. 输出格式仅支持RGB888( `V4L2_PIX_FMT_RGB24` )
	4. 支持设置( `VIDIOC_S_FMT` )和获取( `VIDIOC_G_FMT` )当前输出格式
	5. 支持枚举指定格式的分辨率( `ioctl(VIDIOC_ENUM_FRAMESIZES)` )
	6. 分辨率仅支持640x480
	7. 支持设置和获取当前格式的分辨率
- 设备特性：
	1. 支持在加载模块时指定视频帧的填充颜色
本驱动将作为 `video_device` 的唯一一个不依赖平台总线的驱动。

具体实现应当优先参考代码及其注释，本文仅作补充。

## 3.2 程序基础框架

在本驱动中，主要依赖于如下几个组件的互相配合：
1. `v4l2_device` 基础容器
2. `video_device` ：主要提供了摄像头文件的VFS与ioctl接口
3. `vb2_queue` ：主要提供了缓冲区管理相关机制，包括：
	1. 缓冲区分配、查询、入队、出队等功能
	2. 流开启与关闭功能

## 3.3 资源管理

### 3.3.1 资源分配顺序

在模块的初始化函数中，有如下的资源分配顺序：
1. 分配 `dev` (kzalloc)
2. 初始化设备： `device_initialize`
3. 设置设备名称： `dev_set_name`
4. 注册设备： `device_add`
5. 注册v4l2设备： `v4l2_device_register`
6. 配置video_device：复制 `mvideo_videodev`，设置v4l2_dev等
7. 注册video设备：`video_register_device`
8. 初始化vb2队列：`vb2_queue_init`

### 3.3.2 release回调

在本驱动中，主要有如下三个release回调：
1. `device_release` ：当基础设备模型( `dev->dev` )计数器归0时自动调用
2. `video_device_release` ：取消注册 `video_device` 且设备计数器归0时自动调用
3. `mvideo_release` ：当 `/dev/video*` 文件的打开计数器归0时自动调用

#### 3.3.2.1 mvideo_release

```C

```


#### 3.3.2.2 video_device_release



#### 3.3.2.3 device_release

```C
/**
 * @brief: 基础设备模型释放函数
 * @note: 调用时机为dev的引用计数器变0
 * @param dev
 */
static void device_release(struct device *dev)
{
    kfree(container_of(dev, struct mvideo_dev, dev));
    printk(KERN_INFO "mvideo device released.\n");
}
```

该函数会在基础设备模型计数器归0时自动调用，即此时应当为资源回收的最后一步，清除设备对象的内存即可。

## 3.4 测试运行

```shell
sudo modprobe ./minimal_video_device.ko color_r=255 color_g=0 color_b=0
```

# 4 八股



