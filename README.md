---
number headings: auto, first-level 1, max 6, 1.1
---
# 1 最小Linux驱动

## 1.1 Readme

本仓库为各需求下~~最简~~Linux驱动的实现，尽可能地保持正确和详细。
可配合[h13-0/Notebooks](https://github.com/h13-0/Notebooks)使用。

## 1.2 编译方法

### 1.2.1 基于当前内核和头文件包

基于当前内核和头文件包的编译应按照如下的步骤进行：
1. 确保如下的包已被安装(可使用 `apt` 等工具直接安装)：
	- `linux-headers-$(uname -r)`
2. 进入本仓库根目录，运行 `make`
3. 进入各源文件对应目录，即可找到对应的 `*.ko` 文件
测试方法可见各驱动对应的 `*.md` 文件。

### 1.2.2 从源码重新编译内核并安装





