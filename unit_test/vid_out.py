"""
V4L2 图像生成与输出测试程序
使用 OpenCV 生成动态测试图案并写入 V4L2 设备
"""

import cv2
import numpy as np
import time
import os
import fcntl
import mmap
import v4l2
import errno

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
        print(f"Set format: {width}x{height}, pixel format: {pixel_format:08x} for type {buf_type}")
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
        # 忽略非阻塞模式下的资源不可用错误
        if e.errno == errno.EAGAIN:
            return None
        else:
            print(f"Failed to dequeue buffer: {e}")
            raise e

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
        x = np.linspace(0, 1, width)
        y = np.linspace(0, 1, height)
        xx, yy = np.meshgrid(x, y)

        r = (np.sin(xx * 2 * np.pi + frame_count * 0.05) * 0.5 + 0.5) * 255
        g = (np.sin(yy * 2 * np.pi + frame_count * 0.03) * 0.5 + 0.5) * 255
        b = (np.sin((xx + yy) * np.pi + frame_count * 0.02) * 0.5 + 0.5) * 255

        img = np.stack([r, g, b], axis=2).astype(np.uint8)

    elif pattern_type == "checkerboard":
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
        img = np.zeros((height, width, 3), dtype=np.uint8)
        box_size = 50
        pos = (frame_count * 5) % (width - box_size)
        img[height//2-box_size//2:height//2+box_size//2,
            pos:pos+box_size] = (255, 0, 0)

    else:
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

    # 请求输出缓冲区
    output_buf_count = request_buffers(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, v4l2.V4L2_MEMORY_MMAP, 4)
    if output_buf_count == 0:
        os.close(fd)
        return

    # 查询并映射输出缓冲区
    output_buffers = []
    for i in range(output_buf_count):
        buf = query_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, v4l2.V4L2_MEMORY_MMAP, i)
        if buf is None:
            os.close(fd)
            return

        try:
            # 使用 prot=mmap.PROT_WRITE 确保只用于写入
            mm = mmap.mmap(fd, buf.length, offset=buf.m.offset, flags=mmap.MAP_SHARED, prot=mmap.PROT_WRITE)
            mem_array = np.frombuffer(mm, dtype=np.uint8)
            output_buffers.append((buf, mm, mem_array))
        except Exception as e:
            print(f"Failed to map output buffer {i}: {e}")
            os.close(fd)
            return

    # 预先将所有缓冲区入队
    for i, (buf, _, mem_array) in enumerate(output_buffers):
        test_pattern = generate_test_pattern("color_bars", WIDTH, HEIGHT, i)
        rgb_data = cv2.cvtColor(test_pattern, cv2.COLOR_BGR2RGB)
        mem_array[:] = rgb_data.flatten()

        if not queue_buffer(fd, buf):
            for _, mm, _ in output_buffers:
                mm.close()
            os.close(fd)
            return

    # 启动流传输
    if not stream_on(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT):
        for _, mm, _ in output_buffers:
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

            # 从输出队列取出一个已消费的缓冲区
            output_buf = dequeue_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, v4l2.V4L2_MEMORY_MMAP)
            if output_buf is None:
                # 缓冲区不可用，等待一小段时间后继续
                time.sleep(0.005)
                continue

            # 获取对应的内存映射
            buf_idx = output_buf.index
            buf, mm, mem_array = output_buffers[buf_idx]

            # 生成新的测试图案
            test_pattern = generate_test_pattern(pattern_type, WIDTH, HEIGHT, frame_count)

            # 将图像数据复制到缓冲区
            rgb_data = cv2.cvtColor(test_pattern, cv2.COLOR_BGR2RGB)
            mem_array[:] = rgb_data.flatten()

            # 将缓冲区重新入队
            if not queue_buffer(fd, output_buf):
                break

            # 显示生成的图像
            cv2.imshow('Generated Pattern', test_pattern)

            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('n'):
                pattern_type = pattern_types[pattern_idx]
                pattern_idx = (pattern_idx + 1) % len(pattern_types)
                print(f"Switching to pattern: {pattern_type}")

            frame_count += 1

    except KeyboardInterrupt:
        print("Interrupted by user")

    # 停止流传输
    stream_off(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT)

    # 强制将所有缓冲区出队，以便安全地解除映射
    print("Draining buffers before exit...")
    for i in range(len(output_buffers)):
        try:
            dequeue_buffer(fd, v4l2.V4L2_BUF_TYPE_VIDEO_OUTPUT, v4l2.V4L2_MEMORY_MMAP)
        except IOError:
            # 如果没有缓冲区可出队，忽略该错误
            continue

    # 关闭所有映射的缓冲区
    for _, mm, _ in output_buffers:
        mm.close()

    # 关闭设备
    os.close(fd)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
