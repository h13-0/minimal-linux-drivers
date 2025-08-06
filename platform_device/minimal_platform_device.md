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
	