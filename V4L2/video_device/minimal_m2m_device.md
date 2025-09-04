---
number headings: auto, first-level 1, max 6, 1.1
---
# 1 最小m2m驱动示例

虽然linux内核也提供了一个m2m驱动示例。

本示例依赖系统模块 `v4l2-mem2mem` 和 `videobuf2-vmalloc`

# 2 目录

```toc
```

# 3 特性与实现

## 3.1 特性

该示例拥有如下特性：
- 驱动特性：
	1. 设备基于平台设备实现，其结构为：
		- 平台驱动：静态管理、单例模式。
		- 平台设备：静态管理、单例模式。挂载于 
			- 最小m2m设备对象(`struct mm2m_dev`)
				- v4l2容器
				- 视频设备
	2. 
- V4L2特性：
	1. 
- 设备特性：
	1. 输出方向(内核->用户)支持的像素格式、分辨率等完全取决于输入方向(用户->内核)
	2. 在输入方向完成像素格式和分辨率等设置之前，输出方向的枚举和设置功能均无效

## 3.2 实现

### 3.2.1 资源管理

基本原则：谁创建谁释放
资源从属关系：
- 模块资源注册及顺序：
	1. 注册平台设备
	2. 注册平台驱动，且其 `probe` 中注册：
		1. `mm2m_dev` 对象
		2. 注册v4l2顶层容器
		3. 注册video设备，不过由于是虚拟设备，因此不需要注册资源
		4. 注册m2m实例

因此其资源释放顺序为由下到上，由内到外，其各释放回调<span style="background:#fff88f">已高光标记</span>：
- <span style="background:#fff88f">模块资源释放</span>及顺序：
	1. 释放平台驱动，且<span style="background:#fff88f">其</span> `remove_new` <span style="background:#fff88f">中释放</span>：
		1. 释放m2m实例
		2. 释放video设备，其<span style="background:#fff88f">释放函数</span>不需要释放任何资源
		3. 释放v4l2顶层容器
		4. 释放 `mm2m_dev` 对象
	2. 释放平台设备
