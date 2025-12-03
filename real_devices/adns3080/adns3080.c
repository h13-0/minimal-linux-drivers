#define ADNS3080_DEV_NAME "adns3080"

#ifndef pr_fmt
#define pr_fmt(fmt) ADNS3080_DEV_NAME ": " fmt
#endif

#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/kfifo.h>
#include <linux/printk.h>
#include <linux/delay.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include <linux/spi/spi.h>

#define RETRY_TIMES        (3)                           /** 通信错误的重试次数 */
#define FAULTY_TIMES       (3)                           /** 硬件错误上限，超过该阈值后标记该硬件，并放弃除注销设备外的调用 */
#define FIFO_SIZE          (VIDEO_MAX_FRAME)             /** 必须为2的整数幂，且大于等于2 */



/**
 * @brief: ADNS-3080寄存器组
 */
enum adns3080_regs {
    REG_PRODUCT_ID			= 0x00, /** 产品ID，只读，默认值：0x17 */
    REG_REVISION_ID			= 0x01, /** 修订ID，只读，默认值：0xNN */

    REG_MOTION				= 0x02, /** 运动状态，只读，默认值：0x00 */
    REG_DELTA_X				= 0x03, /** X轴位移，只读，默认值：0x00 */
    REG_DELTA_Y				= 0x04, /** Y轴位移，只读，默认值：0x00 */

    REG_SQUAL				= 0x05, /** 表面质量，只读，默认值：0x00 */
    REG_PIXEL_SUM			= 0x06, /** 像素总和，只读，默认值：0x00 */
    REG_MAXIMUM_PIXEL		= 0x07, /** 最大像素值，只读，默认值：0x00 */

    REG_RESERVED_08			= 0x08, /** 保留寄存器 */
    REG_RESERVED_09			= 0x09, /** 保留寄存器 */

    REG_CONFIGURATION_BITS	= 0x0a, /** 配置位，读写，默认值：0x09 */
    REG_EXTENDED_CONFIG		= 0x0b, /** 扩展配置，读写，默认值：0x00 */

    REG_DATA_OUT_LOWER		= 0x0c, /** 数据输出低字节，只读 */
    REG_DATA_OUT_UPPER		= 0x0d, /** 数据输出高字节，只读 */

    REG_SHUTTER_LOWER		= 0x0e, /** 快门时间低字节，只读，默认值：0x85 */
    REG_SHUTTER_UPPER		= 0x0f, /** 快门时间高字节，只读，默认值：0x00 */

    REG_FRAME_PERIOD_LOWER	= 0x10, /** 帧周期低字节，只读 */
    REG_FRAME_PERIOD_UPPER	= 0x11, /** 帧周期高字节，只读 */

    REG_MOTION_CLEAR		= 0x12, /** 运动清除，只写 */
    REG_FRAME_CAPTURE		= 0x13, /** 帧捕获控制，读写，默认值：0x00 */
    REG_SROM_ENABLE			= 0x14, /** SROM使能，只写，默认值：0x00 */

    REG_RESERVED_15			= 0x15, /** 保留寄存器 */
    REG_RESERVED_16			= 0x16, /** 保留寄存器 */
    REG_RESERVED_17			= 0x17, /** 保留寄存器 */
    REG_RESERVED_18			= 0x18, /** 保留寄存器 */

    REG_FRAME_PERIOD_MAX_BOUND_LOWER	= 0x19, /** 帧周期最大边界低字节，读写，默认值：0xE0 */
    REG_FRAME_PERIOD_MAX_BOUND_UPPER	= 0x1a, /** 帧周期最大边界高字节，读写，默认值：0x2E */
    REG_FRAME_PERIOD_MIN_BOUND_LOWER	= 0x1b, /** 帧周期最小边界低字节，读写，默认值：0x7E */
    REG_FRAME_PERIOD_MIN_BOUND_UPPER	= 0x1c, /** 帧周期最小边界高字节，读写，默认值：0x0E */

    REG_SHUTTER_MAX_BOUND_LOWER		= 0x1d, /** 快门最大边界低字节，读写，默认值：0x00 */
    REG_SHUTTER_MAX_BOUND_UPPER		= 0x1e, /** 快门最大边界高字节，读写，默认值：0x20 */

    REG_SROM_ID				= 0x1f, /** SROM ID，只读，默认值：0x00 */

    /* 0x20-0x3c 为保留寄存器区域 */
    REG_RESERVED_20			= 0x20, /** 保留寄存器 */
    REG_RESERVED_21			= 0x21, /** 保留寄存器 */
    REG_RESERVED_22			= 0x22, /** 保留寄存器 */
    REG_RESERVED_23			= 0x23, /** 保留寄存器 */
    REG_RESERVED_24			= 0x24, /** 保留寄存器 */
    REG_RESERVED_25			= 0x25, /** 保留寄存器 */
    REG_RESERVED_26			= 0x26, /** 保留寄存器 */
    REG_RESERVED_27			= 0x27, /** 保留寄存器 */
    REG_RESERVED_28			= 0x28, /** 保留寄存器 */
    REG_RESERVED_29			= 0x29, /** 保留寄存器 */
    REG_RESERVED_2A			= 0x2a, /** 保留寄存器 */
    REG_RESERVED_2B			= 0x2b, /** 保留寄存器 */
    REG_RESERVED_2C			= 0x2c, /** 保留寄存器 */
    REG_RESERVED_2D			= 0x2d, /** 保留寄存器 */
    REG_RESERVED_2E			= 0x2e, /** 保留寄存器 */
    REG_RESERVED_2F			= 0x2f, /** 保留寄存器 */
    REG_RESERVED_30			= 0x30, /** 保留寄存器 */
    REG_RESERVED_31			= 0x31, /** 保留寄存器 */
    REG_RESERVED_32			= 0x32, /** 保留寄存器 */
    REG_RESERVED_33			= 0x33, /** 保留寄存器 */
    REG_RESERVED_34			= 0x34, /** 保留寄存器 */
    REG_RESERVED_35			= 0x35, /** 保留寄存器 */
    REG_RESERVED_36			= 0x36, /** 保留寄存器 */
    REG_RESERVED_37			= 0x37, /** 保留寄存器 */
    REG_RESERVED_38			= 0x38, /** 保留寄存器 */
    REG_RESERVED_39			= 0x39, /** 保留寄存器 */
    REG_RESERVED_3A			= 0x3a, /** 保留寄存器 */
    REG_RESERVED_3B			= 0x3b, /** 保留寄存器 */
    REG_RESERVED_3C			= 0x3c, /** 保留寄存器 */

    REG_OBSERVATION			= 0x3d, /** 观察寄存器，读写，默认值：0x00 */
    REG_RESERVED_3E			= 0x3e, /** 保留寄存器 */
    REG_INVERSE_PRODUCT_ID	= 0x3f, /** 反向产品ID，只读，默认值：0xF8 */

    REG_PIXEL_BURST			= 0x40, /** 像素突发读取，只读，默认值：0x00 */
    REG_MOTION_BURST		= 0x50, /** 运动突发读取，只读，默认值：0x00 */
    REG_SROM_LOAD			= 0x60, /** SROM加载，只写 */

    ADNS3080_NUM_REGS		= 0x61  /** 寄存器总数 */
};

typedef enum
{
    PPI_400    = 0x00,
    PPI_1600   = 0x01,
} adns3080_ppi;


/**
 * @brief: adns3080设备对象
 */
struct adns3080_dev {
    struct spi_device       *spi;               /** spi设备对象 */
    struct v4l2_device      v4l2_dev;           /** V4L2顶级设备容器 */
    struct video_device     vfd;                /** video设备 */
    struct vb2_queue        queue;              /** vb2缓冲区队列 */

    /**  */
    struct workqueue_struct *workqueue;         /** 使用驱动独享工作队列，避免占用过多的共享资源 */
    struct delayed_work     work;               /** 可延迟工作对象， */

    /** 设备状态属性 */
    uint32_t   		        clock_freq;         /** adns内部时钟速率，默认24Mhz */
    uint16_t                max_frame_period;
    uint16_t                min_frame_period;

    /** 视频设备相关 */
    uint8_t                 scale;              /** 分辨率的放大倍数，adns3080原生分辨率为30x30，最终驱动输出的分辨率为(30*scale)x(30*scale) */

    /** 等待填入图像数据的缓冲区队列相关 */
    DECLARE_KFIFO(fifo, struct vb2_buffer *, FIFO_SIZE);  /** 待填充图像队列 */
    struct spinlock         fifo_spin;          /** 队列自旋锁，用于device_work、stop_streaming、adns3080_remove对队列的互斥 */

    /** 通信错误统计 */
    atomic_t                error_cnt;          /** 通信错误次数计数器，当失败时累加，成功时清零 */
    atomic_t                running;            /** 运行状态标识符，用于避免被多次打开 */
    bool                    enable_fixed_fr:1;  /**  */
    bool                    faulty:1;           /** 硬件错误标识，当触发错误次数大于阈值时标记此位，随后不再处理该硬件 */
};

/**
 * @brief: 向寄存器写入单个字节
 * @note: 该函数包含错误重试机制，也可能导致休眠
 * @return:
 *      - 返回值小于0时表示通信失败
 */
static inline int adns3080_write_reg(struct adns3080_dev *dev, uint8_t reg, uint8_t data)
{
    int ret = 0;
    int i = 0;

    uint8_t tx_buffer[2] = { reg | 0x80, data };

    // 通信与重试机制
    for(; i < RETRY_TIMES ; i++)
    {
        ret = spi_write(dev->spi, tx_buffer, 2);
        if(ret)
        {
            dev_warn(&dev->spi->dev, "write reg 0x%02x failed (%d), retry %d\\n", reg, ret, i+1);
            usleep_range(1000, 2000);
        } else {
            break;
        }
    }

    // 统计错误次数
    if(ret && atomic_inc_return(&dev->error_cnt) > FAULTY_TIMES)
        dev->faulty = true;
    else
        atomic_set(&dev->error_cnt, 0);
    return ret;
}

/**
 * @brief: 从寄存器读取单个字节
 * @note: 该函数包含错误重试机制，也可能导致休眠
 * @return:
 *      - 返回值小于0时表示通信失败
 *      - 返回值大于等于0时标识通信成功，且值为返回值
 */
static inline int adns3080_read_reg(struct adns3080_dev *dev, uint8_t addr)
{
    int ret = 0;
    int i = 0;

    addr &= 0x7f;
    // 通信与重试机制
    for(; i < RETRY_TIMES ; i++)
    {
        ret = spi_w8r8(dev->spi, addr);
        if(ret < 0)
        {
            dev_warn(&dev->spi->dev, "read reg 0x%02x failed (%d), retry %d\\n", addr, ret, i+1);
            usleep_range(1000, 2000);
        } else {
            break;
        }
    }

    // 统计错误次数
    if(ret && atomic_inc_return(&dev->error_cnt) > FAULTY_TIMES)
        dev->faulty = true;
    else
        atomic_set(&dev->error_cnt, 0);
    return ret;
}

/**
 * @brief: 向寄存器的指定位置写入单个字节
 * @note:
 *      - 要求该寄存器可读写
 *      - 该函数包含错误重试机制，也可能导致休眠
 * @return:
 *      - 返回值小于0时表示通信失败
 */
static inline int adns3080_write_reg_bit(struct adns3080_dev *dev, uint8_t addr, uint8_t pos, bool data)
{
    int ret = adns3080_read_reg(dev, REG_CONFIGURATION_BITS);
    if(ret >= 0)
    {
        const uint8_t mask = 1 << pos;
        uint8_t value = ret & 0xff;
        value &= ~mask;
        value |= (data & 0x01) << pos;
        ret = adns3080_write_reg(dev, addr, value);
    }
    return ret;
}

/**
 * @brief: 获取adns3080的Product ID
 * @return: Product ID
 */
int adns3080_get_pid(struct spi_device *spi)
{
    return spi_w8r8(spi, REG_PRODUCT_ID);
}

/**
 * @brief: 设置ADNS-3080分辨率
 */
void adns3080_set_resolution(struct adns3080_dev *dev, adns3080_ppi ppi)
{
    adns3080_write_reg_bit(dev, REG_CONFIGURATION_BITS, 4, ppi);
}

/**
 * @brief: 选择是否开启固定帧率，关闭后帧率会固定为 `Frame_Period_Max_Bound` 中的值
 * @note: 硬件默认为非固定帧率
 */
void adns3080_enable_fixed_fr(struct adns3080_dev *dev, bool enable)
{
    adns3080_write_reg_bit(dev, REG_EXTENDED_CONFIG, 0, enable);
    dev->enable_fixed_fr = enable;
}

/**
 * @brief: 设置最大帧周期
 * @note:
 * 		- 最低帧率 = 时钟速度 / 最大帧周期。
 * 		- 当开启固定帧率时，最大帧周期决定帧率。
 * 		- 默认值为 12000
 */
void adns3080_set_max_frame_period(struct adns3080_dev *dev, uint16_t period)
{
    adns3080_write_reg(dev, REG_FRAME_PERIOD_MAX_BOUND_LOWER, period & 0xff);
    adns3080_write_reg(dev, REG_FRAME_PERIOD_MAX_BOUND_UPPER, (period << 8) & 0xff);
    dev->max_frame_period = period;
}

/**
 * @brief: 设置最小帧周期
 * @note:
 * 		- 最高帧率 = 时钟速度 / 最大帧周期。
 * 		- 当开启固定帧率时，最大帧周期决定帧率。
 * 		- 默认值为 3710
 */
void adns3080_set_min_frame_period(struct adns3080_dev *dev, uint16_t period)
{
    adns3080_write_reg(dev, REG_FRAME_PERIOD_MIN_BOUND_LOWER, period & 0xff);
    adns3080_write_reg(dev, REG_FRAME_PERIOD_MIN_BOUND_UPPER, (period << 8) & 0xff);
    dev->min_frame_period = period;
}

/**
 * @brief: 通过开启固定帧率来设置目标帧率
 * @note: 在容许的范围内，帧率会大于等于目标帧率
 */
void adns3060_set_frame_rate(struct adns3080_dev *dev, uint16_t fps)
{
    uint16_t period = dev->spi->max_speed_hz / fps; // C语言默认只保留整数，即向下取整
    adns3080_enable_fixed_fr(dev, false);
    adns3080_set_max_frame_period(dev, period);
    adns3080_set_min_frame_period(dev, period);
}

/**
 * @brief: 读取指定范围的帧数据
 * @note:
 * 		- adns的分辨率为30x30，因此前900个数据为单个完整帧
 * 		- adns最大可读1536个pixel，多余的我也不知道是什么
 */
int adns3080_cap_frame(struct adns3080_dev *dev, uint8_t *data, int len)
{
    // 提取定义变量兼容老内核的C89
    uint8_t reg = 0x00;

    if(len <= 0 || len > 1536)
        return -EINVAL;

    // 捕获帧
    adns3080_write_reg(dev, REG_FRAME_CAPTURE, 0x83);

    // 等待最大帧周期
    msleep((dev->max_frame_period / dev->spi->max_speed_hz * 1000) + 1);

    // 读取帧
    reg = REG_PIXEL_BURST & 0x7f;

    spi_write(dev->spi, &reg, 1);
    spi_read(dev->spi, data, len);

    return 0;
}

/**
 * @brief: adns3080视频设备的上下文实例
 */
struct adns3080_ctx {
    struct v4l2_fh fh;         /** V4L2的通用文件管理句柄，用于V4L2内部管理用户态的上下文实例 */

    // 设备指针
    struct adns3080_dev *dev;


};

/**
 * @brief: 从文件指针中获取adns3080 video设备的上下文实例
 * @param file: fops传来的文件指针
 * @return: 上下文实例指针
 */
static inline struct adns3080_ctx *file2ctx(struct file *file)
{
    return container_of(file->private_data, struct adns3080_ctx, fh);
}

/**
 * @brief: 从延迟工作对象获取设备指针
 * @param work: 延迟工作对象指针
 * @return: 设备指针
 */
static inline struct adns3080_dev *work2dev(struct work_struct* work)
{
    return container_of(work, struct adns3080_dev, work.work);
}

/**
 * @brief: 延迟任务回调函数
 * @param w
 */
static void device_work(struct work_struct *w)
{
    // 获取当前dev
    struct adns3080_dev* dev = work2dev(w);
    int i = 0, j = 0, x = 0, y = 0, ret = 0;
    // 待填充帧数量
    int buffer_num = 0;
    // 待填充帧数组(由dev->fifo出队获得)
    struct vb2_buffer *buffers[FIFO_SIZE] = { NULL };
    // 原始帧数据(30x30像素)
    static uint8_t raw_data[900] = { 0 };
    // 目标帧平面地址
    uint8_t *dst = NULL;
    // 目标帧尺寸
    uint16_t dst_size = 0;
    // 当前目标帧的行列地址，临时指针
    uint8_t *src_row = NULL, *dst_row = NULL, *p = NULL;

    // 将需要填充数据的缓冲区出队
    spin_lock(&dev->fifo_spin);
    buffer_num = kfifo_out(&dev->fifo, buffers, FIFO_SIZE);
    spin_unlock(&dev->fifo_spin);

    // 执行数据获取与帧生成
    for (i = 0; i < buffer_num; ++i) {
        // 1. 取帧地址
        dst = vb2_plane_vaddr(buffers[i], 0);
        if(dst == NULL)
        {
            vb2_buffer_done(buffers[i], VB2_BUF_STATE_ERROR);
            continue;
        }

        // 2. 获取原始帧数据
        ret = adns3080_cap_frame(dev, raw_data, sizeof(raw_data));
        if (ret != 0) {
            vb2_buffer_done(buffers[i], VB2_BUF_STATE_ERROR);
            continue;
        }

        // 3. 针对dev->scale=1的fast path
        if (dev->scale == 1) {
            memcpy(dst, raw_data, sizeof(raw_data));
            vb2_set_plane_payload(buffers[i], 0, sizeof(raw_data));
            vb2_buffer_done(buffers[i], VB2_BUF_STATE_DONE);
            continue;
        }

        // 4. 计算目标帧尺寸
        dst_size = 30 * max(dev->scale, (uint8_t)1);

        // 5. 执行放大逻辑
        for (y = 0; y < 30; ++y) {
            src_row = raw_data + y * 30;
            // 5.1 第 y 组放大后首行
            dst_row = dst + (y * dev->scale) * dst_size;

            // 5.2 横向放大一行：30 像素 -> 30*scale 像素
            p = dst_row;
            for (x = 0; x < 30; ++x) {
                memset(p, src_row[x], dev->scale);   /* 把一个像素值复制 scale 次 */
                p += dev->scale;
            }

            // 5.3 纵向复制：把这行复制成 scale 行
            for (j = 1; j < dev->scale; ++j)
                memcpy(dst_row + j * dst_size, dst_row, dst_size);
        }

        // 6. 填充帧信息
        vb2_set_plane_payload(buffers[i], 0, dst_size * dst_size);
        vb2_buffer_done(buffers[i], VB2_BUF_STATE_DONE);
    }
}

/**
 * @brief: 队列配置回调
 * @param vq: 需要配置的vb2缓冲区指针
 * @param num_buffers: 驱动所需要的缓冲区数量
 * @param num_planes: 驱动所需平面数量
 * @param sizes: 存储每个平面所需字节数
 * @param alloc_devs: 存储每个平面所分配的设备
 * @return: 0 if success.
 */
static int queue_setup(struct vb2_queue *vq, const void *parg, unsigned int *num_buffers, unsigned int *num_planes,
                       unsigned int sizes[], void *alloc_ctxs[])
{
    struct adns3080_dev *dev = vq->drv_priv;
    int width = dev->scale * 30;
    int height = width;

    if(*num_planes == 0) {
        // 首次调用，此时驱动进行参数配置
        // 配置为单平面
        *num_planes = 1;
        // 每个平面的总字节数为 w*h*1
        sizes[0] = width * height * 1;
        // 确保缓冲区数量，此时 `num_buffers` 会介于 [0, VIDEO_MAX_FRAME] 之间
        v4l2_info(&dev->v4l2_dev, "Initial num_buffers is:%d\n", *num_buffers);
        if(*num_buffers < 3) {
            *num_buffers = 3;
        }
    } else {
        // 后续调用，校验参数是否正确
        if(*num_planes != 1 || sizes[0] != width * height * 1) {
            return -EINVAL;
        }
    }
    return 0;
}

/**
 * @brief: 用户态将缓冲区入队后的回调。
 *      用户空间使用 `VIDIOC_QBUF` 后，缓冲区加入队列之后的回调。在此回调中驱动应当启动硬件操作，
 *      并在帧填充完毕后使用 `vb2_buffer_done` 通知框架。
 * @param vb:
 */
static void buf_queue(struct vb2_buffer *vb)
{
    struct adns3080_dev *dev = vb->vb2_queue->drv_priv;

    spin_lock(&dev->fifo_spin);
    kfifo_put(&dev->fifo, vb);
    spin_unlock(&dev->fifo_spin);

    queue_delayed_work(dev->workqueue, &dev->work, 0);
}


/**
 * @brief: 启动流传输回调。
 *      在普通驱动中应当完成：
 *      1. 确保硬件有足够的缓冲区开始工作
 *      2. 初始化硬件并启动数据流
 *      3. 当驱动程序发生错误时，已经被 `buf_queue` 取出的缓冲区需要通过 `vb2_buffer_done` 来标记和归还缓冲区。
 *          需要注意，其应当将缓冲区标记为 `VB2_BUF_STATE_QUEUED` 。
 *      在本驱动中不需要处理任何事情
 * @param q:
 * @param count: 执行该回调时已经入队的缓冲区数量
 * @return
 */
static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct adns3080_dev *dev = vq->drv_priv;

    spin_lock(&dev->fifo_spin);
    if(!kfifo_is_empty(&dev->fifo))
        queue_delayed_work(dev->workqueue, &dev->work, 0);
    spin_unlock(&dev->fifo_spin);
    return 0;
}


/**
 * @brief: 停止流传输回调
 *      在普通驱动中应当完成：
 *      1. 停止所有硬件传输，确保不再访问所有缓冲区
 *      2. 返回已留给驱动的缓冲区到 `DEQUEUED` 状态从而方便内核及用户态安全释放资源
 *          - 具体而言，此时缓冲区可能有如下几种状态：
 *          - `QUEUED` ：已经入队但还未被驱动处理的缓冲区
 *          - `ACTIVE` ：硬件正在处理的缓冲区
 *          - `DONE` ：已经被驱动处理完成但还未被用户态取走的缓冲区
 *          - `DEQUEUED` ：已经被用户态取走的缓冲区
 *          上述若干状态中，需要处理的缓冲区状态为 `ACTIVE` ，其均需要通过 `vb2_buffer_done` 返回到 `DEQUEUED` ，
 *          且需要注意：
 *          - `ACTIVE` 状态必须返回为 `ERROR` 状态，因为实际上该缓冲区并未正确填充，即：
 *              - `vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);`
 *      3. 手动调用 `vb2_ops.buf_finish` 进行后处理(如果实现了该函数的话)
 * @param vq: 所停止的vb2队列
 */
static void stop_streaming(struct vb2_queue *vq)
{
    struct adns3080_dev *dev = vq->drv_priv;
    // 清空FIFO中的缓冲区
    struct vb2_buffer *vb;
    spin_lock(&dev->fifo_spin);
    while (kfifo_get(&dev->fifo, &vb)) {
        vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
    }
    spin_unlock(&dev->fifo_spin);

    vb2_wait_for_all_buffers(vq);
}


/**
 * @brief: videobuf2的操作回调
 * @note: 只实现必要的回调
 */
static const struct vb2_ops adns3080_video_qops = {
        .queue_setup     = queue_setup,           /** 队列配置回调 */
        .buf_queue       = buf_queue,             /** 用户态将缓冲区入队后的回调 */
        .start_streaming = start_streaming,       /** 启动流传输回调 */
        .stop_streaming  = stop_streaming,        /** 停止流传输回调 */
};

/**
 * @brief: 查询设备能力回调(ioctl(VIDIOC_QUERYCAP, ...))
 */
static int vidioc_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
    strscpy(cap->driver, ADNS3080_DEV_NAME, sizeof(cap->driver));
    strscpy(cap->card, ADNS3080_DEV_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "NULL:%s", ADNS3080_DEV_NAME);
    return 0;
}

/**
 * @brief: 枚举作为video_capture时支持的格式
 * @note:
 * @param file:
 * @param fh:
 * @param f:
 * @return
 */
static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
    struct adns3080_ctx* ctx = fh;
    struct adns3080_dev *dev = ctx->dev;

    v4l2_info(&dev->v4l2_dev, "enum videoc fmt cap at index:%d\n", f->index);
    if(f->index == 0)
    {
        f->pixelformat = V4L2_PIX_FMT_GREY;
        return 0;
    } else {
        return -EINVAL;
    }
}

/**
 * @brief: 查询video_capture的当前格式
 * @param file: 用户空间操作的文件句柄
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    struct adns3080_ctx* ctx = fh;
    struct adns3080_dev *dev = ctx->dev;
    f->fmt.pix.width         = dev->scale * 30;     /** 分辨率为(scale*30)x(scale*30) */
    f->fmt.pix.height        = dev->scale * 30;
    f->fmt.pix.field         = V4L2_FIELD_NONE;     /** 使用普通逐行扫描 */
    f->fmt.pix.pixelformat   = V4L2_PIX_FMT_GREY;   /** 使用灰度数据格式 */
    return 0;
}

/**
 * @brief: 设置video_capture的数据格式
 * @note:
 *      adns3080原生分辨率为30x30，因此本驱动支持对原生图像整数倍向上放大。
 *      文档规定：除非 `f->type` 字段错误，否则不应返回错误代码，因此直接修改格式和参数即可。
 * @param file: 用户空间操作的文件句柄
 * @param fh:
 * @param f: 用于返回给用户空间的格式信息
 * @return
 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    struct adns3080_ctx* ctx = fh;
    struct adns3080_dev *dev = ctx->dev;
    int scale                = max(f->fmt.pix.width, f->fmt.pix.height) / 30; /** 分辨率对30向下取整 */

    dev->scale               = max(scale, 1);                                 /** 最小分辨率大于30 */
    f->fmt.pix.width         = dev->scale * 30;                               /** 分辨率为(scale*30)x(scale*30) */
    f->fmt.pix.height        = dev->scale * 30;
    f->fmt.pix.field         = V4L2_FIELD_NONE;                               /** 使用普通逐行扫描，大多数CMOS均为此格式 */
    f->fmt.pix.pixelformat   = V4L2_PIX_FMT_GREY;                             /** 使用灰度数据格式 */
    return 0;
}

/**
 * @brief: 枚举分辨率的回调函数(ioctl(VIDIOC_ENUM_FRAMESIZES))
 * @param file:
 * @param fh:
 *      通常指向上下文实例的 `struct v4l2_fh` 成员 。实际上指向的是 `mvideo_open` 函数中设置的 `file->private_data` 的值。
 *      但是通常会把 `struct v4l2_fh` 放到上下文实例(`adns3080_ctx`)的第一个成员中，因此通常二者地址相同。
 * @param fsize: 返回给用户空间的分辨率信息
 * @return
 */
static int vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
    // 仅支持一个步进分辨率
    if(fsize->index != 0) {
        return -EINVAL;
    }

    // 仅支持GRAY
    if(fsize->pixel_format != V4L2_PIX_FMT_GREY) {
        return -EINVAL;
    }

    // 设置分辨率类型为步进类型
    fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
    fsize->stepwise.min_width = 30;    /** 最小分辨率宽度为30 */
    fsize->stepwise.min_height = 30;
    fsize->stepwise.step_width = 30;   /** 分辨率宽度步进为30 */
    fsize->stepwise.step_height = 30;
    fsize->stepwise.max_width = 7650;  /** 最大分辨率宽度为30*255 */
    fsize->stepwise.max_height = 7650;
    return 0;
}

/**
 * @brief: ioctl操作函数表
 */
static const struct v4l2_ioctl_ops adns3080_video_ioctl_ops = {
    .vidioc_querycap         = vidioc_querycap,          /** 查询设备能力 */
    .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,  /** 枚举video_capture支持的格式(ioctl(VIDIOC_ENUM_FMT)) */
    .vidioc_g_fmt_vid_cap    = vidioc_g_fmt_vid_cap,     /** 查询video_capture的当前格式 */
    .vidioc_s_fmt_vid_cap    = vidioc_s_fmt_vid_cap,     /** 设置video_capture的数据格式 */
    .vidioc_enum_framesizes  = vidioc_enum_framesizes,   /** 枚举分辨率 */

    .vidioc_reqbufs          = vb2_ioctl_reqbufs,        /** 使用videobuf2提供的机制 */
    .vidioc_querybuf         = vb2_ioctl_querybuf,
    .vidioc_qbuf             = vb2_ioctl_qbuf,
    .vidioc_dqbuf            = vb2_ioctl_dqbuf,
    .vidioc_streamon         = vb2_ioctl_streamon,
    .vidioc_streamoff        = vb2_ioctl_streamoff,
};


/**
 * @brief: 视频设备文件打开接口(/dev/videoX)
 * @param file: 文件指针
 * @return: 0 if success.
 */
static int adns3080_video_open(struct file *file)
{
    int ret = 0;
    struct adns3080_ctx *ctx = NULL;
    struct adns3080_dev *dev = video_drvdata(file);

    if(atomic_read(&dev->running))
        return -EBUSY;

    // 分配上下文实例
    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if(!ctx) {
        ret = -ENOMEM;
        goto err_alloc_ctx;
    }
    ctx->dev = dev;

    // 初始化文件句柄
    v4l2_fh_init(&ctx->fh, video_devdata(file));

    // 将上下文的文件句柄存入private_data
    // 注意不是存自己定义的上下文实例，因为V4L2内部实现是依赖于其文件句柄
    file->private_data = &ctx->fh;

    // 添加文件句柄到设备列表
    v4l2_fh_add(&ctx->fh);

    atomic_inc(&dev->running);

err_alloc_ctx:
    return ret;
}

/**
 * @brief: 设备释放函数，当打开计数器归零时被调用
 * @param file: 文件指针
 * @return: 0 if success.
 */
static int adns3080_video_release(struct file *file)
{
    struct adns3080_ctx *ctx = file2ctx(file);
    struct adns3080_dev *dev = ctx->dev;
    v4l2_fh_del(&ctx->fh);
    v4l2_fh_exit(&ctx->fh);
    kfree(ctx);
    atomic_dec(&dev->running);
    return 0;
}

/**
 * @brief: VFS操作接口
 */
static const struct v4l2_file_operations adns3080_video_fops = {
    .owner          = THIS_MODULE,              /** 所有者指向本模块 */
    .open           = adns3080_video_open,      /** 设备打开回调 */
    .release        = adns3080_video_release,   /** 设备释放回调 */
    .unlocked_ioctl = video_ioctl2,
    .mmap           = vb2_fop_mmap,
};

/**
 * @brief: video设备的释放接口
 * @note:
 *      触发流程：adns3080_exit -> spi_unregister_driver -> adns3080_remove -> video_unregister_device
 *          -> adns3080_video_device_release
 *      资源管理原则：谁注册谁释放，video_device并没有注册资源，所以不需要释放资源
 * @param vdev:
 */
static void adns3080_video_device_release(struct video_device *vdev)
{
}

/**
 * @brief: 视频设备模型相关配置
 */
static const struct video_device adns3080_videodev = {
    .name        = ADNS3080_DEV_NAME,                             /** 暴露于用户空间的设备名 */
    .vfl_dir     = VFL_DIR_RX,                                    /** 定义为接收设备(用户态视角接收数据) */
    .device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE ,  /** 设备能力注册为流设备和捕获设备 */
    .fops        = &adns3080_video_fops,                          /** VFS操作接口 */
    .ioctl_ops   = &adns3080_video_ioctl_ops,                     /** ioctl操作函数表 */
    .release     = adns3080_video_device_release,                 /** 设备释放接口 */
};

/**
 * @brief: spi设备匹配及初始化函数
 * @param spi: 要操作的spi设备
 * @return: 0 if succ.
 */
static int adns3080_probe(struct spi_device *spi)
{
    // 0. 定义变量以兼容老内核的C89标准
    int ret = 0;
    int adns3080_pid = 0;
    struct adns3080_dev *dev = NULL;
    struct video_device *vfd = NULL;

    // 1. 重启adns3080
    //adns3080_reset(dev);
    msleep(1);

    // 2. 校验ID
    adns3080_pid = adns3080_get_pid(spi);
    if(adns3080_pid < 0)
    {
        pr_err("SPI transmission failed while obtaining ADNS3080 PID, abort.\n");
        ret = -EIO;
        goto err_get_pid;
    } else if(adns3080_pid != 0x17) {
        pr_err("ADNS3080 PID mismatch, expecting 0x17 but receiving 0x%02x, abort.\n", adns3080_pid & 0xff);
        ret = -ENXIO;
        goto err_get_pid;
    } else {
        pr_info("ADNS3080 PID match success.\n");
    }

    // 3. 实例化设备
    // 3.1 使用devm_kzalloc分配内存并托管到系统设备链表，可省去当设备移除或驱动解绑时的资源释放操作
    dev = devm_kzalloc(&spi->dev, sizeof(struct adns3080_dev), GFP_KERNEL);
    if(dev == NULL)
    {
        ret = -ENOMEM;
        goto err_devm_kzalloc;
    }
    // 3.2 寄存数据
    dev->spi = spi;
    spi_set_drvdata(spi, (void*)dev);
    // 3.3 初始化自旋锁
    spin_lock_init(&dev->fifo_spin);

    // 4. 开启固定帧率
    adns3080_enable_fixed_fr(dev, false);
    dev->max_frame_period = 0x2ee0;
    dev->min_frame_period = 0x0dac;

    // 5. 初始化v4l2容器
    ret = v4l2_device_register(&dev->spi->dev, &dev->v4l2_dev);
    if(ret) {
        goto err_v4l2_register;
    }

    // 配置视频设备模型
    dev->vfd = adns3080_videodev;
    vfd = &dev->vfd;
    vfd->v4l2_dev = &dev->v4l2_dev;

    // 向video_device中寄存私有数据
    video_set_drvdata(vfd, dev);

    // 6. 注册video设备
    ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
    if(ret) {
        goto err_video_register;
    }

    // 7. 初始化vb2队列
    dev->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;                   // 指定为视频捕获设备
    dev->queue.io_modes = VB2_MMAP | VB2_USERPTR;
    dev->queue.drv_priv = dev;
    dev->queue.ops = &adns3080_video_qops;
    dev->queue.mem_ops = &vb2_vmalloc_memops;                        // 指定使用vmalloc内存
    dev->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    ret = vb2_queue_init(&dev->queue);
    if(ret) {
        goto err_vb2_init;
    }
    dev->vfd.queue = &dev->queue;

    // 8. 初始化延迟工作队列，用于执行捕获帧生成
    dev->workqueue = create_singlethread_workqueue("adns3080 workqueue");
    if(dev->workqueue == NULL)
    {
        goto err_queue_init;
    }

    // 初始化延迟工作对象，并设置回调函数
    INIT_DEFERRABLE_WORK(&dev->work, device_work);

    // success.
    return ret;

    // next err
    // 取消所有任务并释放延迟工作队列
    // cancel_delayed_work_sync(&dev->work);
    // destroy_workqueue(dev->workqueue);

err_queue_init:
    vb2_queue_release(&dev->queue);

err_vb2_init:
    video_unregister_device(&dev->vfd);

err_video_register:
    v4l2_device_unregister(&dev->v4l2_dev);

err_v4l2_register:
    // 由于使用devm_kzalloc，因此只需要返回错误码，而无须手动释放设备内存
    // kfree(dev);
err_devm_kzalloc:
err_get_pid:
    return ret;
}

/**
 * @brief: spi驱动解绑回调函数
 * @note: 在新版本内核中返回值为void，在老版本内核中为int
 * @param spi: 需要解绑的spi设备
 */
static int adns3080_remove(struct spi_device *spi)
{
    struct adns3080_dev *dev = (struct adns3080_dev *)spi_get_drvdata(spi);

    // 倒序注销资源
    // 取消所有任务
    cancel_delayed_work_sync(&dev->work);
    destroy_workqueue(dev->workqueue);
    vb2_queue_release(&dev->queue);
    video_unregister_device(&dev->vfd);
    v4l2_device_unregister(&dev->v4l2_dev);
    // 当使用devm_kzalloc时，则无须再手动释放设备对象，只需要完成上述软硬件操作即可
    // kfree(dev);

    return 0;
}

/**
 * @brief: 设备树兼容性列表
 */
static const struct of_device_id adns3080_dt_ids[] = {
    { .compatible = "avago,adns-3080" },                 /** 不可加空格。当且仅当compatible字符串与设备树完全一致时才可匹配 */
    {},                                                  /** 设备兼容性列表必须以空接点结尾 */
};
MODULE_DEVICE_TABLE(of, adns3080_dt_ids);

/**
 * @brief: adns3080 SPI驱动结构体
 * @note:
 *      - spi设备的匹配优先级为：
 *          1. 用户覆盖
 *          2. 设备树匹配
 *          3. ACPI匹配
 *          4. spi_driver.id_table匹配
 *          5. 名称匹配
 *        在本驱动中使用设备树匹配方式
 */
static struct spi_driver adns3080_driver = {
    .probe = adns3080_probe,
    .remove = adns3080_remove,
    .driver = {
        .name = ADNS3080_DEV_NAME,
        .of_match_table = adns3080_dt_ids,
    },
};

/**
 * @brief: 模块卸载函数，只需要注销spi驱动即可
 */
static void __exit adns3080_exit(void)
{
    spi_unregister_driver(&adns3080_driver);
}

/**
 * @brief: 模块加载函数，只需要注册spi驱动即可
 */
static int __init adns3080_init(void)
{
    return spi_register_driver(&adns3080_driver);
}

MODULE_LICENSE("GPL v2");
module_init(adns3080_init);
module_exit(adns3080_exit);
