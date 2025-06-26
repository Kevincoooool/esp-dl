#ifndef _CAMERA_TEST_H_
#define _CAMERA_TEST_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "linux/videodev2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 图像格式类型定义
 */
typedef enum {
    CAMERA_TEST_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    CAMERA_TEST_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    CAMERA_TEST_FMT_GREY = V4L2_PIX_FMT_GREY,
    CAMERA_TEST_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    CAMERA_TEST_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    CAMERA_TEST_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    CAMERA_TEST_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} camera_test_fmt_t;

/**
 * @brief 初始化摄像头
 * 
 * @param fmt 期望的图像格式
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t camera_test_init(camera_test_fmt_t fmt);

/**
 * @brief 获取一帧JPEG图像
 * 
 * @param jpeg_buffer 输出缓冲区指针的指针，函数会设置该指针指向JPEG图像数据
 * @param jpeg_size 输出参数，返回JPEG图像数据大小
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t camera_test_get_jpeg(uint8_t **jpeg_buffer, size_t *jpeg_size);

/**
 * @brief 获取摄像头分辨率
 * 
 * @param width 输出参数，返回宽度
 * @param height 输出参数，返回高度
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t camera_test_get_resolution(uint32_t *width, uint32_t *height);

/**
 * @brief 停止摄像头
 * 
 * @return esp_err_t ESP_OK表示成功，其他表示失败
 */
esp_err_t camera_test_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // _CAMERA_TEST_H_ 