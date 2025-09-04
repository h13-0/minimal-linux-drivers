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

## 3.3 测试运行

python可视化运行：

```python
#!/usr/bin/env python3
"""
V4L2 M2M 设备测试程序
使用 OpenCV 生成测试图案并通过 V4L2 M2M 设备进行传输和捕获
"""

import cv2
import numpy as np
import time
import os
import sys
import fcntl
import mmap
import v4l2  # 需要安装 v4l2-python3 包: pip install v4l2-python3

# V4L2 设备路径
V4L2_DEVICE = "/dev/video0"

# 图像尺寸
WIDTH = 640
HEIGHT = 480

# 像素格式 (RGB24)
PIXEL_FORMAT = v4l2.V4L2_PIX_FMT_RGB24

def set_v4l2_format(fd, width, height, pixel_format, buf_type):
    """设置 V4L2 设备格式"""
    fmt = v4l2.v4l2_format()
    fmt.type = buf_type
    fmt.fmt.pix.width = width
    fmt.fmt.pix.height = height
    fmt.fmt.pix.pixelformat = pixel_format
    fmt.fmt.pix.field = v4l2.V4L2_FIELD_NONE
    
    try:
        fcntl.ioctl(fd, v4l2.VIDIOC_S_FMT, fmt)
        print(f"Set format: {width}x{height}, pixel format: {pixel_format:08x}")
        return True
    except IOError as e:
        print(f"Failed to set format: {e}")
        return False

def request_buffers(fd, buf_type, memory, count):
    """请求 V4L2 缓冲区"""
    req = v4l2.v4l2_requestbuffers()
    req.count = count
    req.type = buf_type
    req.memory = memory
    
    try:
        fcntl.ioctl(fd, v4l2.VIDIOC_REQBUFS, req)
        print(f"Requested {req.count} buffers for type {buf_type}")
        return req.count
    except IOError as e:
        print(f"Failed to request buffers: {e}")
        return 0

def query_buffer(fd, buf_type, memory, index):
    """查询缓冲区信息"""
    buf = v4l2.v4l2_buffer()
    buf.type = buf_type
    buf.memory = memory
    buf.index = index
    
    try:
        fcntl.ioctl(fd, v4l2.VIDIOC_QUERYBUF, buf)
        return buf
    except IOError as e:
        print(f"Failed to query buffer: {e}")
        return None

def queue_buffer(fd, buf):
    """将缓冲区加入队列"""
    try:
        fcntl.ioctl(fd, v4l2.VIDIOC_QBUF, buf)
        return True
    except IOError as e:
        print(f"Failed to queue buffer: {e}")
        return False

def dequeue_buffer(fd, buf_type, memory):
    """从队列中取出缓冲区"""
    buf = v4l2.v4l2_buffer()
    buf.type = buf_type
    buf.memory = memory
    
    try:
        fcntl.ioctl(fd, v4l2.VIDIOC_DQBUF, buf)
        return buf
    except IOError as e:
        print(f"Failed to dequeue buffer: {e}")
        return None

def stream_on(fd, buf_type):
    """启动流传输"""
    try:
        buf_type_ptr = np.array([buf_type], dtype=np.int32)
        fcntl.ioctl(fd, v4l2.VIDIOC_STREAMON, buf_type_ptr)
        print(f"Stream ON for type {buf_type}")
        return True
    except IOError as e:
        print(f"Failed to start streaming: {e}")
        return False

def stream_off(fd, buf_type):
    """停止流传输"""
    try:
        buf_type_ptr = np.array([buf_type], dtype=np.int32)
        fcntl.ioctl(fd, v4l2.VIDIOC_STREAMOFF, buf_type_ptr)
        print(f"Stream OFF for type {buf_type}")
        return True
    except IOError as e:
        print(f"Failed to stop streaming: {e}")
        return False

def generate_test_pattern(pattern_type, width, height, frame_count):
    """生成测试图案"""
    if pattern_type == "color_bars":
        # 生成彩色条
        img = np.zeros((height, width, 3), dtype=np.uint8)
        bar_width = width // 8
        colors = [
            (255, 255, 255),  # 白
            (255, 255, 0),    # 黄
            (0, 255, 255),    # 青
            (0, 255, 0),      # 绿
            (255, 0, 255),    # 品红
            (255, 0, 0),      # 红
            (0, 0, 255),      # 蓝
            (0, 0, 0),        # 黑
        ]
        
        for i, color in enumerate(colors):
            start_x = i * bar_width
            end_x = (i + 1) * bar_width
            if end_x > width:
                end_x = width
            img[:, start_x:end_x] = color
    
    elif pattern_type == "gradient":
        # 生成渐变图案
        x = np.linspace(0, 1, width)
        y = np.linspace(0, 1, height)
        xx, yy = np.meshgrid(x, y)
        
        r = (np.sin(xx * 2 * np.pi + frame_count * 0.05) * 0.5 + 0.5) * 255
        g = (np.sin(yy * 2 * np.pi + frame_count * 0.03) * 0.5 + 0.5) * 255
        b = (np.sin((xx + yy) * np.pi + frame_count * 0.02) * 0.5 + 0.5) * 255
        
        img = np.stack([r, g, b], axis=2).astype(np.uint8)
    
    elif pattern_type == "checkerboard":
        # 生成棋盘格图案
        tile_size = 40
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        for y in range(0, height, tile_size):
            for x in range(0, width, tile_size):
                if (x // tile_size + y // tile_size) % 2 == 0:
                    color = (255, 255, 255)
                else:
                    color = (0, 0, 0)
                img[y:y+tile_size, x:x+tile_size] = color
    
    elif pattern_type == "moving_box":
        # 生成移动方块
        img = np.zeros((height, width, 3), dtype=np.uint8)
        box_size = 50
        pos = (frame_count * 5) % (width - box_size)
        img[height//2-box_size//2:height//2+box_size//2, 
            pos:pos+box_size] = (255, 0, 0)
    
    else:
        # 默认生成彩色噪声
        img = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
    
    return img

def main():
    # 检查设备是否存在
    if not os.path.exists(V4L2_DEVICE):
        print(f"V4L2 device {V4L2_DEVICE} not found!")
        return
    
    # 打开设备
    try:
        fd = os.open(V4L2_DEVICE, os.O_RDWR | os.O_NONBLOCK)
    except OSError as e:
        print(f"Failed to open device {V4L2_DEVICE}: {e}")
        return
    
    # 设置输出队列格式
    if not set_v4l2_format(fd, WIDTH, HEIGHT, PIXEL_FORMAT, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT):
        os.close(fd)
        return
    
    # 设置捕获队列格式
    if not set_v4l2_format(fd, WIDTH, HEIGHT, PIXEL_FORMAT, v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE):
        os.close(fd)
        return
    
    # 请求输出缓冲区
    output_buf_count = request_buffers(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, 
                                      v4l2.V4L2_MEMORY_MMAP, 4)
    if output_buf_count == 0:
        os.close(fd)
        return
    
    # 请求捕获缓冲区
    capture_buf_count = request_buffers(fd, v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE, 
                                       v4l2.V4L2_MEMORY_MMAP, 4)
    if capture_buf_count == 0:
        os.close(fd)
        return
    
    # 查询并映射输出缓冲区
    output_buffers = []
    for i in range(output_buf_count):
        buf = query_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, 
                          v4l2.V4L2_MEMORY_MMAP, i)
        if buf is None:
            os.close(fd)
            return
        
        # 使用 mmap 映射内存
        try:
            mm = mmap.mmap(fd, buf.length, offset=buf.m.offset, 
                          flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)
            # 将 mmap 对象转换为 numpy 数组
            mem_array = np.frombuffer(mm, dtype=np.uint8)
            output_buffers.append((buf, mm, mem_array))
        except Exception as e:
            print(f"Failed to map output buffer {i}: {e}")
            os.close(fd)
            return
    
    # 查询并映射捕获缓冲区
    capture_buffers = []
    for i in range(capture_buf_count):
        buf = query_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE, 
                          v4l2.V4L2_MEMORY_MMAP, i)
        if buf is None:
            os.close(fd)
            return
        
        # 使用 mmap 映射内存
        try:
            mm = mmap.mmap(fd, buf.length, offset=buf.m.offset, 
                          flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)
            # 将 mmap 对象转换为 numpy 数组
            mem_array = np.frombuffer(mm, dtype=np.uint8)
            capture_buffers.append((buf, mm, mem_array))
        except Exception as e:
            print(f"Failed to map capture buffer {i}: {e}")
            # 清理已映射的输出缓冲区
            for _, mm, _ in output_buffers:
                mm.close()
            os.close(fd)
            return
    
    # 将所有缓冲区加入队列
    for buf, mm, mem_array in output_buffers:
        if not queue_buffer(fd, buf):
            # 清理已映射的缓冲区
            for _, mm, _ in output_buffers + capture_buffers:
                mm.close()
            os.close(fd)
            return
    
    for buf, mm, mem_array in capture_buffers:
        if not queue_buffer(fd, buf):
            # 清理已映射的缓冲区
            for _, mm, _ in output_buffers + capture_buffers:
                mm.close()
            os.close(fd)
            return
    
    # 启动流传输
    if not stream_on(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT):
        # 清理已映射的缓冲区
        for _, mm, _ in output_buffers + capture_buffers:
            mm.close()
        os.close(fd)
        return
    
    if not stream_on(fd, v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE):
        stream_off(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT)
        # 清理已映射的缓冲区
        for _, mm, _ in output_buffers + capture_buffers:
            mm.close()
        os.close(fd)
        return
    
    # 测试图案类型列表
    pattern_types = ["color_bars", "gradient", "checkerboard", "moving_box", "noise"]
    pattern_idx = 0
    frame_count = 0
    
    print("Press 'q' to quit, 'n' to switch to next pattern")
    
    try:
        while True:
            # 每100帧切换一次图案
            if frame_count % 100 == 0:
                pattern_type = pattern_types[pattern_idx]
                pattern_idx = (pattern_idx + 1) % len(pattern_types)
                print(f"Switching to pattern: {pattern_type}")
            
            # 生成测试图案
            test_pattern = generate_test_pattern(pattern_type, WIDTH, HEIGHT, frame_count)
            
            # 从输出队列取出一个缓冲区
            output_buf = dequeue_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, 
                                       v4l2.V4L2_MEMORY_MMAP)
            if output_buf is None:
                break
            
            # 将测试图案复制到输出缓冲区
            buf_idx = output_buf.index
            buf, mm, mem_array = output_buffers[buf_idx]
            
            # 将图像数据复制到缓冲区
            rgb_data = cv2.cvtColor(test_pattern, cv2.COLOR_BGR2RGB)
            mem_array[:] = rgb_data.flatten()
            
            # 将缓冲区重新加入队列
            if not queue_buffer(fd, output_buf):
                break
            
            # 从捕获队列取出一个缓冲区
            capture_buf = dequeue_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE, 
                                        v4l2.V4L2_MEMORY_MMAP)
            if capture_buf is None:
                break
            
            # 获取捕获的数据
            buf_idx = capture_buf.index
            buf, mm, mem_array = capture_buffers[buf_idx]
            
            # 将捕获的数据转换为图像
            captured_data = mem_array.copy().reshape((HEIGHT, WIDTH, 3))
            captured_img = cv2.cvtColor(captured_data, cv2.COLOR_RGB2BGR)
            
            # 将缓冲区重新加入队列
            if not queue_buffer(fd, capture_buf):
                break
            
            # 显示原始图像和捕获的图像
            cv2.imshow('Original Pattern', test_pattern)
            cv2.imshow('Captured Frame', captured_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('n'):
                pattern_type = pattern_types[pattern_idx]
                pattern_idx = (pattern_idx + 1) % len(pattern_types)
                print(f"Switching to pattern: {pattern_type}")
            
            frame_count += 1
            time.sleep(0.033)  # 约30fps
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    # 停止流传输
    stream_off(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT)
    stream_off(fd, v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
    
    # 关闭所有映射的缓冲区
    for _, mm, _ in output_buffers + capture_buffers:
        mm.close()
    
    # 关闭设备
    os.close(fd)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

C语言简单测试： ^j7ak9n

```C
#include <linux/videodev2.h>  
#include <fcntl.h>  
#include <sys/ioctl.h>  
#include <sys/mman.h>  
#include <stdio.h>  
#include <string.h>  
#include <errno.h>  
#include <unistd.h>  
  
const char* device = "/dev/video0";

int main(int argc, char **argv)  
{  
    int fd = open(device, O_RDWR);  
    if (fd < 0) {  
        printf("open device failed: %s\n", strerror(errno));  
        return -1;  
    }  
  
    // 设置输出队列格式  
    struct v4l2_format fmt_out = {0};  
    fmt_out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;  
    fmt_out.fmt.pix.width = 1920;  
    fmt_out.fmt.pix.height = 1080;  
    fmt_out.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24; // RGB3  
    printf("try to set fmt_out.pix.width=%d\r\n", fmt_out.fmt.pix.width);  
    printf("try to set fmt_out.pix.height=%d\r\n", fmt_out.fmt.pix.height);  
    printf("try to set fmt_out.pix.pixelformat=%c.%c.%c.%c\r\n",  
           fmt_out.fmt.pix.pixelformat >> 0  & 0xff,  
           fmt_out.fmt.pix.pixelformat >> 8  & 0xff,  
           fmt_out.fmt.pix.pixelformat >> 16 & 0xff,  
           fmt_out.fmt.pix.pixelformat >> 24 & 0xff  
    );  
    if (ioctl(fd, VIDIOC_S_FMT, &fmt_out) < 0) {  
        printf("set output format failed: %s\n", strerror(errno));  
        close(fd);  
        return -1;  
    }  
  
    // 需要检查实际分辨率等信息  
    printf("get fmt_out.pix.width=%d\r\n", fmt_out.fmt.pix.width);  
    printf("get fmt_out.pix.height=%d\r\n", fmt_out.fmt.pix.height);  
    printf("get fmt_out.pix.height=%d\r\n", fmt_out.fmt.pix.height);  
    printf("get fmt_out.pix.pixelformat=%c.%c.%c.%c\r\n",  
           fmt_out.fmt.pix.pixelformat >> 0  & 0xff,  
           fmt_out.fmt.pix.pixelformat >> 8  & 0xff,  
           fmt_out.fmt.pix.pixelformat >> 16 & 0xff,  
           fmt_out.fmt.pix.pixelformat >> 24 & 0xff  
    );  
  
    // 设置捕获队列格式（实际上会与输出相同，但需要显式设置）  
    struct v4l2_format fmt_cap = {0};  
    fmt_cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  
    if (ioctl(fd, VIDIOC_S_FMT, &fmt_cap) < 0) {  
        printf("set capture format failed: %s\n", strerror(errno));  
        close(fd);  
        return -1;  
    }  
    printf("get fmt_cap.pix.width=%d\r\n", fmt_cap.fmt.pix.width);  
    printf("get fmt_cap.pix.height=%d\r\n", fmt_cap.fmt.pix.height);  
    printf("get fmt_cap.pix.height=%d\r\n", fmt_cap.fmt.pix.height);  
    printf("get fmt_cap.pix.pixelformat=%c.%c.%c.%c\r\n",  
           fmt_cap.fmt.pix.pixelformat >> 0  & 0xff,  
           fmt_cap.fmt.pix.pixelformat >> 8  & 0xff,  
           fmt_cap.fmt.pix.pixelformat >> 16 & 0xff,  
           fmt_cap.fmt.pix.pixelformat >> 24 & 0xff  
    );  
  
    // 申请输出队列缓冲区  
    struct v4l2_requestbuffers req_out = {0};  
    req_out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;  
    req_out.count = 4;  
    req_out.memory = V4L2_MEMORY_MMAP;  
    if (ioctl(fd, VIDIOC_REQBUFS, &req_out) < 0) {  
        printf("request output buffers failed: %s\n", strerror(errno));  
        close(fd);  
        return -1;  
    }  
    // 需要注意获得的缓冲区数量不等于申请到的缓冲区数量  
    printf("The number of obtained output buffers=%d.\r\n", req_out.count);  
  
    // 申请捕获队列缓冲区  
    struct v4l2_requestbuffers req_cap = {0};  
    req_cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  
    req_cap.count = 4;  
    req_cap.memory = V4L2_MEMORY_MMAP;  
    if (ioctl(fd, VIDIOC_REQBUFS, &req_cap) < 0) {  
        printf("request capture buffers failed: %s\n", strerror(errno));  
        close(fd);  
        return -1;  
    }  
    // 需要注意获得的缓冲区数量不等于申请到的缓冲区数量  
    printf("The number of obtained capture buffers=%d.\r\n", req_cap.count);  
  
    // 映射输出队列缓冲区  
    struct v4l2_buffer buf_out = {0};  
    void *out_buffers[req_out.count];  
    for (int i = 0; i < req_out.count; i++) {  
        buf_out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;  
        buf_out.memory = V4L2_MEMORY_MMAP;  
        buf_out.index = i;  
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf_out) < 0) {  
            printf("query output buffer failed: %s\n", strerror(errno));  
            close(fd);  
            return -1;  
        }  
        out_buffers[i] = mmap(NULL, buf_out.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf_out.m.offset);  
        if (out_buffers[i] == MAP_FAILED) {  
            printf("mmap output buffer failed: %s\n", strerror(errno));  
            close(fd);  
            return -1;  
        }  
        // 入队输出缓冲区  
        if (ioctl(fd, VIDIOC_QBUF, &buf_out) < 0) {  
            printf("queue output buffer failed: %s\n", strerror(errno));  
            close(fd);  
            return -1;  
        }  
    }  
  
    // 映射捕获队列缓冲区  
    struct v4l2_buffer buf_cap = {0};  
    void *cap_buffers[req_cap.count];  
    for (int i = 0; i < req_cap.count; i++) {  
        buf_cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  
        buf_cap.memory = V4L2_MEMORY_MMAP;  
        buf_cap.index = i;  
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf_cap) < 0) {  
            printf("query capture buffer failed: %s\n", strerror(errno));  
            close(fd);  
            return -1;  
        }  
        cap_buffers[i] = mmap(NULL, buf_cap.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf_cap.m.offset);  
        if (cap_buffers[i] == MAP_FAILED) {  
            printf("mmap capture buffer failed: %s\n", strerror(errno));  
            close(fd);  
            return -1;  
        }  
        // 入队捕获缓冲区  
        if (ioctl(fd, VIDIOC_QBUF, &buf_cap) < 0) {  
            printf("queue capture buffer failed: %s\n", strerror(errno));  
            close(fd);  
            return -1;  
        }  
    }  
  
    // 启动输出队列流传输  
    enum v4l2_buf_type type_out = V4L2_BUF_TYPE_VIDEO_OUTPUT;  
    if (ioctl(fd, VIDIOC_STREAMON, &type_out) < 0) {  
        printf("streamon output failed: %s\n", strerror(errno));  
        close(fd);  
        return -1;  
    }  
  
    // 启动捕获队列流传输  
    enum v4l2_buf_type type_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;  
    if (ioctl(fd, VIDIOC_STREAMON, &type_cap) < 0) {  
        printf("streamon capture failed: %s\n", strerror(errno));  
        close(fd);  
        return -1;  
    }  
  
    // 循环处理缓冲区  
    while (1) {  
        // 出队捕获缓冲区（处理后的数据）  
        buf_cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  
        buf_cap.memory = V4L2_MEMORY_MMAP;  
        if (ioctl(fd, VIDIOC_DQBUF, &buf_cap) < 0) {  
            printf("dequeue capture buffer failed: %s\n", strerror(errno));  
            break;  
        }  
        printf("Got capture buffer index %d\n", buf_cap.index);  
  
        // 在这里处理捕获缓冲区中的数据（例如保存图像或显示）  
  
        // 重新入队捕获缓冲区  
        if (ioctl(fd, VIDIOC_QBUF, &buf_cap) < 0) {  
            printf("requeue capture buffer failed: %s\n", strerror(errno));  
            break;  
        }  
  
        // 出队输出缓冲区  
        buf_out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;  
        buf_out.memory = V4L2_MEMORY_MMAP;  
        if (ioctl(fd, VIDIOC_DQBUF, &buf_out) < 0) {  
            printf("dequeue output buffer failed: %s\n", strerror(errno));  
            break;  
        }  
        printf("Got output buffer index %d\n", buf_out.index);  
  
        // 在这里填充输出缓冲区中的数据（例如生成新图像）  
  
        // 重新入队输出缓冲区  
        if (ioctl(fd, VIDIOC_QBUF, &buf_out) < 0) {  
            printf("requeue output buffer failed: %s\n", strerror(errno));  
            break;  
        }  
    }  
  
    // 停止流传输和清理资源  
    ioctl(fd, VIDIOC_STREAMOFF, &type_out);  
    ioctl(fd, VIDIOC_STREAMOFF, &type_cap);  
    for (int i = 0; i < req_out.count; i++) {  
        munmap(out_buffers[i], buf_out.length);  
    }  
    for (int i = 0; i < req_cap.count; i++) {  
        munmap(cap_buffers[i], buf_cap.length);  
    }  
    close(fd);  
    return 0;  
}
```