/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "driver/jpeg_encode.h"

// 视频帧缓冲区数量，太大的值可能导致内存分配失败
#define KSDIY_VIDEO_BUFFER_COUNT   2
#define MEMORY_TYPE                V4L2_MEMORY_MMAP

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
#define CAM_DEV_PATH                 ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#elif CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
#define CAM_DEV_PATH                 ESP_VIDEO_DVP_DEVICE_NAME
#else
#error "未选择摄像头传感器"
#endif

#define JPEG_ENC_QUALITY             (80)

/*
 * 摄像头控制结构体
*/
typedef struct ksdiy_camera {
    int fd;                         // 摄像头设备文件描述符
    uint32_t width;                 // 图像宽度
    uint32_t height;                // 图像高度
    uint32_t pixel_format;          // 像素格式
    jpeg_encode_cfg_t jpeg_enc_config;  // JPEG编码配置
    size_t jpeg_enc_output_buf_alloced_size;  // JPEG输出缓冲区分配大小
    jpeg_encoder_handle_t jpeg_handle;  // JPEG编码器句柄
    uint8_t *jpeg_out_buf;          // JPEG输出缓冲区
    uint8_t *buffer[KSDIY_VIDEO_BUFFER_COUNT];  // 视频缓冲区
    bool is_streaming;              // 是否正在流式传输
} ksdiy_camera_t;

/*
 * 图像格式类型定义
 */
typedef enum {
    KSDIY_VIDEO_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    KSDIY_VIDEO_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    KSDIY_VIDEO_FMT_GREY = V4L2_PIX_FMT_GREY,
    KSDIY_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    KSDIY_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    KSDIY_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    KSDIY_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} ksdiy_fmt_t;

static const int s_queue_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
static const char *TAG = "ksdiy_camera";
static ksdiy_camera_t *s_camera = NULL;

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
static const esp_video_init_csi_config_t csi_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port      = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_PORT,
                .scl_pin   = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN,
                .sda_pin   = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN,
            },
            .freq = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_FREQ,
        },
        .reset_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN,
        .pwdn_pin  = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN,
    },
};
#endif

#if CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
static const esp_video_init_dvp_config_t dvp_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port      = CONFIG_EXAMPLE_DVP_SCCB_I2C_PORT,
                .scl_pin   = CONFIG_EXAMPLE_DVP_SCCB_I2C_SCL_PIN,
                .sda_pin   = CONFIG_EXAMPLE_DVP_SCCB_I2C_SDA_PIN,
            },
            .freq      = CONFIG_EXAMPLE_DVP_SCCB_I2C_FREQ,
        },
        .reset_pin = CONFIG_EXAMPLE_DVP_CAM_SENSOR_RESET_PIN,
        .pwdn_pin  = CONFIG_EXAMPLE_DVP_CAM_SENSOR_PWDN_PIN,
        .dvp_pin = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                CONFIG_EXAMPLE_DVP_D0_PIN, CONFIG_EXAMPLE_DVP_D1_PIN, CONFIG_EXAMPLE_DVP_D2_PIN, CONFIG_EXAMPLE_DVP_D3_PIN,
                CONFIG_EXAMPLE_DVP_D4_PIN, CONFIG_EXAMPLE_DVP_D5_PIN, CONFIG_EXAMPLE_DVP_D6_PIN, CONFIG_EXAMPLE_DVP_D7_PIN,
            },
            .vsync_io = CONFIG_EXAMPLE_DVP_VSYNC_PIN,
            .de_io = CONFIG_EXAMPLE_DVP_DE_PIN,
            .pclk_io = CONFIG_EXAMPLE_DVP_PCLK_PIN,
            .xclk_io = CONFIG_EXAMPLE_DVP_XCLK_PIN,
        },
        .xclk_freq = CONFIG_EXAMPLE_DVP_XCLK_FREQ,
    },
};
#endif

static const esp_video_init_config_t cam_config = {
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
    .csi      = csi_config,
#endif
#if CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
    .dvp      = dvp_config,
#endif
};

/**
 * @brief   打开视频设备并初始化视频设备，使用 `init_fmt` 作为输出格式。
 * @note    当传感器以RAW格式输出数据时，ISP模块可以将其数据插值为RGB或YUV格式。
 *          但是，当传感器以RGB或YUV格式工作时，输出数据只能是RGB或YUV格式。
 * @param dev 设备名称(例如, "/dev/video0")
 * @param init_fmt 输出格式。
 *
 * @return
 *     - 设备描述符   成功
 *     - -1 错误
 */
static int ksdiy_video_open(char *dev, ksdiy_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "打开视频设备失败");
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "获取设备能力失败");
        goto exit_0;
    }

    // ESP_LOGI(TAG, "版本: %d.%d.%d", (uint16_t)(capability.version >> 16),
    //          (uint8_t)(capability.version >> 8),
    //          (uint8_t)capability.version);
    // ESP_LOGI(TAG, "驱动:  %s", capability.driver);
    // ESP_LOGI(TAG, "卡:    %s", capability.card);
    // ESP_LOGI(TAG, "总线:  %s", capability.bus_info);

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "获取格式失败");
        goto exit_0;
    }

    // ESP_LOGI(TAG, "宽度=%" PRIu32 " 高度=%" PRIu32, default_format.fmt.pix.width, default_format.fmt.pix.height);

    if (default_format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width = default_format.fmt.pix.width,
            .fmt.pix.height = default_format.fmt.pix.height,
            .fmt.pix.pixelformat = init_fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "设置格式失败");
            goto exit_0;
        }
    }

    return fd;
exit_0:
    close(fd);
    return -1;
}

static jpeg_enc_input_format_t get_jpeg_enc_input_fmt(uint32_t video_fmt)
{
    jpeg_enc_input_format_t ret_fmt = JPEG_ENCODE_IN_FORMAT_YUV422;
    switch (video_fmt) {
    case KSDIY_VIDEO_FMT_YUV422:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_YUV422;
        break;
    case KSDIY_VIDEO_FMT_RAW8: // 将raw8视为灰度，仅用于测试
    case KSDIY_VIDEO_FMT_GREY:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_GRAY;
        break;
    case KSDIY_VIDEO_FMT_RGB565:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_RGB565;
        break;
    case KSDIY_VIDEO_FMT_RGB888:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_RGB888;
        break;
    default:
        ESP_LOGE(TAG, "不支持的格式");
        ret_fmt = -1;
        break;
    }
    return ret_fmt;
}

/**
 * @brief 初始化摄像头
 * 
 * @param fmt 期望的图像格式
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t ksdiy_camera_init(ksdiy_fmt_t fmt)
{
    esp_err_t ret = ESP_OK;
    
    // 初始化摄像头硬件
    ESP_ERROR_CHECK(esp_video_init(&cam_config));
    
    // 分配摄像头控制结构体
    s_camera = calloc(1, sizeof(ksdiy_camera_t));
    if (!s_camera) {
        ESP_LOGE(TAG, "分配摄像头控制结构体失败");
        return ESP_ERR_NO_MEM;
    }
    
    // 打开摄像头设备
    int fd = ksdiy_video_open(CAM_DEV_PATH, fmt);
    if (fd < 0) {
        ESP_LOGE(TAG, "打开摄像头设备失败");
        free(s_camera);
        s_camera = NULL;
        return ESP_FAIL;
    }
    
    s_camera->fd = fd;
    
    // 获取摄像头格式信息
    struct v4l2_format format;
    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(TAG, "获取格式失败");
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_FAIL;
    }
    
    s_camera->width = format.fmt.pix.width;
    s_camera->height = format.fmt.pix.height;
    s_camera->pixel_format = format.fmt.pix.pixelformat;
    
    // 初始化JPEG编码器
    jpeg_enc_input_format_t jpeg_enc_infmt = get_jpeg_enc_input_fmt(format.fmt.pix.pixelformat);
    if (jpeg_enc_infmt == -1) {
        ESP_LOGE(TAG, "不支持的像素格式");
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_FAIL;
    }
    
    s_camera->jpeg_enc_config.src_type = jpeg_enc_infmt;
    s_camera->jpeg_enc_config.image_quality = JPEG_ENC_QUALITY;
    s_camera->jpeg_enc_config.width = format.fmt.pix.width;
    s_camera->jpeg_enc_config.height = format.fmt.pix.height;
    
    size_t jpeg_enc_input_src_size;
    if (s_camera->pixel_format == KSDIY_VIDEO_FMT_RAW8 || s_camera->pixel_format == KSDIY_VIDEO_FMT_GREY) {
        s_camera->jpeg_enc_config.sub_sample = JPEG_DOWN_SAMPLING_GRAY;
        jpeg_enc_input_src_size = format.fmt.pix.width * format.fmt.pix.height;
    } else if (s_camera->pixel_format == KSDIY_VIDEO_FMT_YUV420) {
        s_camera->jpeg_enc_config.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
        jpeg_enc_input_src_size = format.fmt.pix.width * format.fmt.pix.height * 3 / 2;
    } else {
        s_camera->jpeg_enc_config.sub_sample = JPEG_DOWN_SAMPLING_YUV422;
        jpeg_enc_input_src_size = format.fmt.pix.width * format.fmt.pix.height * 2;
    }
    
    // 创建JPEG编码器
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
    };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&encode_eng_cfg, &s_camera->jpeg_handle));
    
    // 分配JPEG输出缓冲区
    jpeg_encode_memory_alloc_cfg_t jpeg_enc_output_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    
    s_camera->jpeg_out_buf = (uint8_t *)jpeg_alloc_encoder_mem(jpeg_enc_input_src_size / 2, 
                                                             &jpeg_enc_output_mem_cfg, 
                                                             &s_camera->jpeg_enc_output_buf_alloced_size);
    if (!s_camera->jpeg_out_buf) {
        ESP_LOGE(TAG, "分配JPEG输出缓冲区失败");
        jpeg_del_encoder_engine(s_camera->jpeg_handle);
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = KSDIY_VIDEO_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = MEMORY_TYPE;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "请求缓冲区失败");
        // jpeg_free_encoder_mem(s_camera->jpeg_out_buf);
        jpeg_del_encoder_engine(s_camera->jpeg_handle);
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_FAIL;
    }
    
    // 映射缓冲区
    for (int i = 0; i < KSDIY_VIDEO_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = MEMORY_TYPE;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "查询缓冲区失败");
            ret = ESP_FAIL;
            goto cleanup;
        }
        
        s_camera->buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                            MAP_SHARED, fd, buf.m.offset);
        if (!s_camera->buffer[i]) {
            ESP_LOGE(TAG, "映射缓冲区失败");
            ret = ESP_FAIL;
            goto cleanup;
        }
        
        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "入队帧缓冲区失败");
            ret = ESP_FAIL;
            goto cleanup;
        }
    }
    
    // 开始流式传输
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type)) {
        ESP_LOGE(TAG, "开始流式传输失败");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    s_camera->is_streaming = true;
    ESP_LOGI(TAG, "摄像头初始化成功，分辨率: %ldx%ld", s_camera->width, s_camera->height);
    return ESP_OK;
    
cleanup:
    for (int i = 0; i < KSDIY_VIDEO_BUFFER_COUNT; i++) {
        if (s_camera->buffer[i]) {
            munmap(s_camera->buffer[i], 0);
        }
    }
    // jpeg_free_encoder_mem(s_camera->jpeg_out_buf);
    jpeg_del_encoder_engine(s_camera->jpeg_handle);
    close(fd);
    free(s_camera);
    s_camera = NULL;
    return ret;
}

/**
 * @brief 获取一帧图像
 * 
 * @param buffer 输出缓冲区指针的指针，函数会设置该指针指向图像数据
 * @param buffer_size 输出参数，返回图像数据大小
 * @param fmt 输出参数，返回图像格式
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t ksdiy_camera_get_frame(uint8_t **buffer, size_t *buffer_size, uint32_t *fmt)
{
    if (!s_camera || !s_camera->is_streaming) {
        ESP_LOGE(TAG, "摄像头未初始化或未开始流式传输");
        return ESP_ERR_INVALID_STATE;
    }
    
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = s_queue_buf_type;
    buf.memory = MEMORY_TYPE;
    
    // 从队列中取出一个缓冲区
    if (ioctl(s_camera->fd, VIDIOC_DQBUF, &buf) != 0) {
        ESP_LOGE(TAG, "获取视频帧失败");
        return ESP_FAIL;
    }
    
    // 设置输出参数
    *buffer = s_camera->buffer[buf.index];
    *buffer_size = buf.bytesused;
    *fmt = s_camera->pixel_format;
    
    // 将缓冲区重新放入队列
    if (ioctl(s_camera->fd, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGE(TAG, "释放视频帧失败");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief 获取一帧JPEG图像
 * 
 * @param jpeg_buffer 输出缓冲区指针的指针，函数会设置该指针指向JPEG图像数据
 * @param jpeg_size 输出参数，返回JPEG图像数据大小
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t ksdiy_camera_get_jpeg(uint8_t **jpeg_buffer, size_t *jpeg_size)
{
    if (!s_camera || !s_camera->is_streaming) {
        ESP_LOGE(TAG, "摄像头未初始化或未开始流式传输");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t *frame_buffer;
    size_t frame_size;
    uint32_t frame_format;
    
    // 获取一帧图像
    esp_err_t ret = ksdiy_camera_get_frame(&frame_buffer, &frame_size, &frame_format);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 如果已经是JPEG格式，直接返回
    if (frame_format == V4L2_PIX_FMT_JPEG) {
        *jpeg_buffer = frame_buffer;
        *jpeg_size = frame_size;
        return ESP_OK;
    }
    
    // 否则进行JPEG编码
    uint32_t jpeg_encoded_size = 0;
    ret = jpeg_encoder_process(s_camera->jpeg_handle, 
                              &s_camera->jpeg_enc_config, 
                              frame_buffer, 
                              frame_size, 
                              s_camera->jpeg_out_buf, 
                              s_camera->jpeg_enc_output_buf_alloced_size, 
                              &jpeg_encoded_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG编码失败");
        return ret;
    }
    
    *jpeg_buffer = s_camera->jpeg_out_buf;
    *jpeg_size = jpeg_encoded_size;
    
    return ESP_OK;
}

/**
 * @brief 获取摄像头分辨率
 * 
 * @param width 输出参数，返回宽度
 * @param height 输出参数，返回高度
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t ksdiy_camera_get_resolution(uint32_t *width, uint32_t *height)
{
    if (!s_camera) {
        ESP_LOGE(TAG, "摄像头未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    *width = s_camera->width;
    *height = s_camera->height;
    
    return ESP_OK;
}

/**
 * @brief 停止摄像头
 * 
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t ksdiy_camera_deinit(void)
{
    if (!s_camera) {
        return ESP_OK;
    }
    
    if (s_camera->is_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(s_camera->fd, VIDIOC_STREAMOFF, &type) != 0) {
            ESP_LOGE(TAG, "停止流式传输失败");
        }
        s_camera->is_streaming = false;
    }
    
    for (int i = 0; i < KSDIY_VIDEO_BUFFER_COUNT; i++) {
        if (s_camera->buffer[i]) {
            munmap(s_camera->buffer[i], 0);
        }
    }
    
    // if (s_camera->jpeg_out_buf) {
    //     jpeg_free_encoder_mem(s_camera->jpeg_out_buf);
    // }
    
    if (s_camera->jpeg_handle) {
        jpeg_del_encoder_engine(s_camera->jpeg_handle);
    }
    
    if (s_camera->fd >= 0) {
        close(s_camera->fd);
    }
    
    free(s_camera);
    s_camera = NULL;
    
    return ESP_OK;
}