#include "camera_test.h"
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

// 视频帧缓冲区数量
#define CAMERA_TEST_BUFFER_COUNT   2
#define MEMORY_TYPE                V4L2_MEMORY_MMAP

// 添加摄像头设备路径常量
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
#define CAM_DEV_PATH                 ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#elif CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
#define CAM_DEV_PATH                 ESP_VIDEO_DVP_DEVICE_NAME
#else
// 默认使用 MIPI CSI 设备路径
#define CAM_DEV_PATH                 "/dev/video0"
#endif

#define JPEG_ENC_QUALITY             (80)

static const char *TAG = "camera_test";

/*
 * 摄像头控制结构体
*/
typedef struct {
    int fd;                         // 摄像头设备文件描述符
    uint32_t width;                 // 图像宽度
    uint32_t height;                // 图像高度
    uint32_t pixel_format;          // 像素格式
    jpeg_encode_cfg_t jpeg_enc_config;  // JPEG编码配置
    size_t jpeg_enc_output_buf_alloced_size;  // JPEG输出缓冲区分配大小
    jpeg_encoder_handle_t jpeg_handle;  // JPEG编码器句柄
    uint8_t *jpeg_out_buf;          // JPEG输出缓冲区
    uint8_t *buffer[CAMERA_TEST_BUFFER_COUNT];  // 视频缓冲区
    bool is_streaming;              // 是否正在流式传输
} camera_test_t;

static camera_test_t *s_camera = NULL;
static const int s_queue_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

// 基本的摄像头配置 - 简化版本，只提供NULL配置
static const esp_video_init_config_t cam_config = {
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
    .csi = NULL,  // 使用默认配置或在实际硬件上需要提供正确的配置
#endif
#if CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
    .dvp = NULL,  // 使用默认配置或在实际硬件上需要提供正确的配置
#endif
};

// 打开摄像头设备（简化版本）
static int camera_test_video_open(const char *path, camera_test_fmt_t fmt) {
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device");
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "Failed to get device capabilities");
        goto exit_0;
    }

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "Failed to get format");
        goto exit_0;
    }

    if (default_format.fmt.pix.pixelformat != fmt) {
        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width = default_format.fmt.pix.width,
            .fmt.pix.height = default_format.fmt.pix.height,
            .fmt.pix.pixelformat = fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "Failed to set format");
            goto exit_0;
        }
    }

    return fd;
exit_0:
    close(fd);
    return -1;
}

static jpeg_enc_input_format_t get_jpeg_enc_input_fmt(uint32_t video_fmt) {
    jpeg_enc_input_format_t ret_fmt = JPEG_ENCODE_IN_FORMAT_YUV422;
    switch (video_fmt) {
    case CAMERA_TEST_FMT_YUV422:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_YUV422;
        break;
    case CAMERA_TEST_FMT_RAW8:
    case CAMERA_TEST_FMT_GREY:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_GRAY;
        break;
    case CAMERA_TEST_FMT_RGB565:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_RGB565;
        break;
    case CAMERA_TEST_FMT_RGB888:
        ret_fmt = JPEG_ENCODE_IN_FORMAT_RGB888;
        break;
    default:
        ESP_LOGE(TAG, "Unsupported format");
        ret_fmt = -1;
        break;
    }
    return ret_fmt;
}

/**
 * @brief 初始化摄像头
 */
esp_err_t camera_test_init(camera_test_fmt_t fmt) {
    esp_err_t ret = ESP_OK;
    
    // 初始化摄像头硬件
    ESP_ERROR_CHECK(esp_video_init(&cam_config));
    
    // 分配摄像头控制结构体
    s_camera = calloc(1, sizeof(camera_test_t));
    if (!s_camera) {
        ESP_LOGE(TAG, "Failed to allocate camera struct");
        return ESP_ERR_NO_MEM;
    }
    
    // 打开摄像头设备
    int fd = camera_test_video_open(CAM_DEV_PATH, fmt);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open camera device");
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
        ESP_LOGE(TAG, "Failed to get format");
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_FAIL;
    }
    
    s_camera->width = format.fmt.pix.width;
    s_camera->height = format.fmt.pix.height;
    s_camera->pixel_format = format.fmt.pix.pixelformat;
    
    ESP_LOGI(TAG, "Camera format: %ldx%ld, pixel_format=0x%lx", 
             s_camera->width, s_camera->height, s_camera->pixel_format);
    
    // 初始化JPEG编码器
    jpeg_enc_input_format_t jpeg_enc_infmt = get_jpeg_enc_input_fmt(format.fmt.pix.pixelformat);
    if (jpeg_enc_infmt == -1) {
        ESP_LOGE(TAG, "Unsupported pixel format");
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
    if (s_camera->pixel_format == CAMERA_TEST_FMT_RAW8 || s_camera->pixel_format == CAMERA_TEST_FMT_GREY) {
        s_camera->jpeg_enc_config.sub_sample = JPEG_DOWN_SAMPLING_GRAY;
        jpeg_enc_input_src_size = format.fmt.pix.width * format.fmt.pix.height;
    } else if (s_camera->pixel_format == CAMERA_TEST_FMT_YUV420) {
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
        ESP_LOGE(TAG, "Failed to allocate JPEG output buffer");
        jpeg_del_encoder_engine(s_camera->jpeg_handle);
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = CAMERA_TEST_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = MEMORY_TYPE;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "Failed to request buffers");
        jpeg_del_encoder_engine(s_camera->jpeg_handle);
        close(fd);
        free(s_camera);
        s_camera = NULL;
        return ESP_FAIL;
    }
    
    // 映射缓冲区
    for (int i = 0; i < CAMERA_TEST_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;
        
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = MEMORY_TYPE;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "Failed to query buffer");
            ret = ESP_FAIL;
            goto cleanup;
        }
        
        s_camera->buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                            MAP_SHARED, fd, buf.m.offset);
        if (!s_camera->buffer[i]) {
            ESP_LOGE(TAG, "Failed to map buffer");
            ret = ESP_FAIL;
            goto cleanup;
        }
        
        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "Failed to queue buffer");
            ret = ESP_FAIL;
            goto cleanup;
        }
    }
    
    // 开始流式传输
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type)) {
        ESP_LOGE(TAG, "Failed to start streaming");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    s_camera->is_streaming = true;
    ESP_LOGI(TAG, "Camera initialized successfully: %ldx%ld", s_camera->width, s_camera->height);
    return ESP_OK;
    
cleanup:
    for (int i = 0; i < CAMERA_TEST_BUFFER_COUNT; i++) {
        if (s_camera->buffer[i]) {
            munmap(s_camera->buffer[i], 0);
        }
    }
    jpeg_del_encoder_engine(s_camera->jpeg_handle);
    close(fd);
    free(s_camera);
    s_camera = NULL;
    return ret;
}

/**
 * @brief 获取一帧图像
 */
static esp_err_t camera_test_get_frame(uint8_t **buffer, size_t *buffer_size, uint32_t *fmt) {
    if (!s_camera || !s_camera->is_streaming) {
        ESP_LOGE(TAG, "Camera not initialized or not streaming");
        return ESP_ERR_INVALID_STATE;
    }
    
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = s_queue_buf_type;
    buf.memory = MEMORY_TYPE;
    
    // 从队列中取出一个缓冲区
    if (ioctl(s_camera->fd, VIDIOC_DQBUF, &buf) != 0) {
        ESP_LOGE(TAG, "Failed to dequeue buffer");
        return ESP_FAIL;
    }
    
    // 设置输出参数
    *buffer = s_camera->buffer[buf.index];
    *buffer_size = buf.bytesused;
    *fmt = s_camera->pixel_format;
    
    // 将缓冲区重新放入队列
    if (ioctl(s_camera->fd, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGE(TAG, "Failed to queue buffer");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief 获取一帧JPEG图像
 */
esp_err_t camera_test_get_jpeg(uint8_t **jpeg_buffer, size_t *jpeg_size) {
    if (!s_camera || !s_camera->is_streaming) {
        ESP_LOGE(TAG, "Camera not initialized or not streaming");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t *frame_buffer;
    size_t frame_size;
    uint32_t frame_format;
    
    // 获取一帧图像
    esp_err_t ret = camera_test_get_frame(&frame_buffer, &frame_size, &frame_format);
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
        ESP_LOGE(TAG, "JPEG encoding failed");
        return ret;
    }
    
    *jpeg_buffer = s_camera->jpeg_out_buf;
    *jpeg_size = jpeg_encoded_size;
    
    return ESP_OK;
}

/**
 * @brief 获取摄像头分辨率
 */
esp_err_t camera_test_get_resolution(uint32_t *width, uint32_t *height) {
    if (!s_camera) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    *width = s_camera->width;
    *height = s_camera->height;
    
    return ESP_OK;
}

/**
 * @brief 停止摄像头
 */
esp_err_t camera_test_deinit(void) {
    if (!s_camera) {
        return ESP_OK;
    }
    
    if (s_camera->is_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(s_camera->fd, VIDIOC_STREAMOFF, &type) != 0) {
            ESP_LOGE(TAG, "Failed to stop streaming");
        }
        s_camera->is_streaming = false;
    }
    
    for (int i = 0; i < CAMERA_TEST_BUFFER_COUNT; i++) {
        if (s_camera->buffer[i]) {
            munmap(s_camera->buffer[i], 0);
        }
    }
    
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