---
number headings: auto, first-level 2, max 6, 1.1
---
#嵌入式 #Linux驱动开发 #V4L2 

## 1 最小video_device示例

## 2 目录

```toc
```

## 3 特性与实现

### 3.1 基本特性

该示例拥有如下特性：
- 基本特性：
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
	8. 

### 3.2 设备能力设置







