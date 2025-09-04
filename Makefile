# 指定内核构建目录
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# 定义所有需要编译的模块
obj-m += platform_device/mplatform_devdrv.o
obj-m += V4L2/video_device/mm2m_device.o
obj-m += V4L2/video_device/mvideo_device.o

# 构建所有模块
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# 清理生成的文件
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: all clean
