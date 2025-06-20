---
number headings: auto, first-level 2, max 6, 1.1
---
#嵌入式 #Linux驱动开发 #V4L2 

## 1 最小video_device示例

## 2 目录

```toc
```

## 3 特性与目标

### 3.1 基本特性

该示例拥有如下特性：
1. 使用最简device结构而不使用平台设备等总线模型
2. 基础总线模型的挂载位置位于 `/sys/devices/mvideo`
3. `v4l2_device` 的挂载位置位于 `/sys/devices/mvideo/v4l2-device`
4. `video` 设备的文件位于 `/dev/videoX`
5. 设备能力设置为 `V4L2_CAP_STREAMING`
6. 

### 3.2 



