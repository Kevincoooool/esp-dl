#include "app_video.h"
#include "coco_pose.hpp"
#include "dl_image.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ksdiy_lvgl_port.h"
#include "lv_demos.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "yolo11n-pose";

// 定义目标分辨率
#define TARGET_WIDTH 224
#define TARGET_HEIGHT 224
#define FRAME_BUFFER_SIZE (TARGET_WIDTH * TARGET_HEIGHT * 3)

// 显示模式控制宏定义
#define CONTINUOUS_REFRESH_MODE     1    // 1: 持续刷新摄像头画面, 0: 检测到人体后暂停刷新
#define FREEZE_ON_DETECTION         1    // 1: 检测到人体时冻结画面保持边框, 0: 边框会被新帧覆盖
#define DETECTION_HOLD_TIME_MS      3000 // 检测到人体后保持画面冻结的时间(毫秒)
#define SHOW_BBOX_ONLY_ON_DETECTION 0    // 1: 只在检测到人体时显示边框, 0: 实时显示检测结果
#define DRAW_SKELETON_LINES         1    // 1: 绘制人体骨骼线条, 0: 只显示关键点

// 全局变量定义
static lv_obj_t *camera_img = NULL;        // LVGL图像对象
static lv_img_dsc_t camera_img_desc;       // 图像描述符
static lv_obj_t *status_label = NULL;      // 状态标签
static lv_obj_t *result_label = NULL;      // 检测结果标签
static QueueHandle_t display_queue = NULL; // 显示队列
static QueueHandle_t detect_queue = NULL;  // 检测队列
static QueueHandle_t result_queue = NULL;  // 结果队列
static bool detection_enabled = true;      // 姿态检测使能标志
static uint32_t frame_count = 0;           // 帧计数器

// 显示控制变量
static bool freeze_display = false;        // 是否冻结显示
static uint32_t last_detection_time = 0;   // 最后检测到人体的时间
static bool has_detection = false;         // 当前是否有检测结果
static uint8_t *frozen_frame_buffer = NULL; // 冻结帧缓冲区

typedef struct {
    uint8_t *buffer;
    uint32_t width;
    uint32_t height;
    size_t size;
    uint32_t format;
} frame_buffer_t;

typedef struct {
    int person_count;
    float confidence;
    bool detected;
    char status_text[64];
} detection_result_t;

// 关键点名称
static const char *kpt_names[17] = {"nose",
                                    "left eye",
                                    "right eye",
                                    "left ear",
                                    "right ear",
                                    "left shoulder",
                                    "right shoulder",
                                    "left elbow",
                                    "right elbow",
                                    "left wrist",
                                    "right wrist",
                                    "left hip",
                                    "right hip",
                                    "left knee",
                                    "right knee",
                                    "left ankle",
                                    "right ankle"};

// 人体骨骼连接定义 (基于COCO 17个关键点)
// 每对数字表示要连接的两个关键点的索引
static const int skeleton_connections[][2] = {
    // 头部连接
    {0, 1},   // nose -> left_eye
    {0, 2},   // nose -> right_eye
    {1, 3},   // left_eye -> left_ear
    {2, 4},   // right_eye -> right_ear
    
    // 躯干连接
    {5, 6},   // left_shoulder -> right_shoulder
    {5, 11},  // left_shoulder -> left_hip
    {6, 12},  // right_shoulder -> right_hip
    {11, 12}, // left_hip -> right_hip
    
    // 左臂连接
    {5, 7},   // left_shoulder -> left_elbow
    {7, 9},   // left_elbow -> left_wrist
    
    // 右臂连接
    {6, 8},   // right_shoulder -> right_elbow
    {8, 10},  // right_elbow -> right_wrist
    
    // 左腿连接
    {11, 13}, // left_hip -> left_knee
    {13, 15}, // left_knee -> left_ankle
    
    // 右腿连接
    {12, 14}, // right_hip -> right_knee
    {14, 16}  // right_knee -> right_ankle
};

static const int num_skeleton_connections = sizeof(skeleton_connections) / sizeof(skeleton_connections[0]);

// 在RGB888图像上绘制直线
void draw_line_rgb888(uint8_t *buffer, int width, int height, int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b, int thickness)
{
    // 边界检查
    if ((x1 < 0 && x2 < 0) || (x1 >= width && x2 >= width) || 
        (y1 < 0 && y2 < 0) || (y1 >= height && y2 >= height)) {
        return; // 线段完全在图像外
    }
    
    // 使用Bresenham算法绘制直线
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    int x = x1, y = y1;
    
    while (true) {
        // 绘制当前点（考虑thickness）
        for (int t = -thickness/2; t <= thickness/2; ++t) {
            for (int tt = -thickness/2; tt <= thickness/2; ++tt) {
                int px = x + t;
                int py = y + tt;
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    int idx = (py * width + px) * 3;
                    buffer[idx] = r;     // R
                    buffer[idx + 1] = g; // G
                    buffer[idx + 2] = b; // B
                }
            }
        }
        
        // 检查是否到达终点
        if (x == x2 && y == y2) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

// 在RGB888图像上绘制矩形边框
void draw_rectangle_rgb888(uint8_t *buffer, int width, int height, int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b, int thickness)
{
    // 边界检查
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= width) x2 = width - 1;
    if (y2 >= height) y2 = height - 1;
    if (x1 > x2) { int tmp = x1; x1 = x2; x2 = tmp; }
    if (y1 > y2) { int tmp = y1; y1 = y2; y2 = tmp; }

    // 绘制水平线（上下边框）
    for (int t = 0; t < thickness; ++t) {
        for (int x = x1; x <= x2; ++x) {
            // 上边框
            if (y1 + t >= 0 && y1 + t < height && x >= 0 && x < width) {
                int idx = ((y1 + t) * width + x) * 3;
                buffer[idx] = r;     // R
                buffer[idx + 1] = g; // G
                buffer[idx + 2] = b; // B
            }
            // 下边框
            if (y2 - t >= 0 && y2 - t < height && x >= 0 && x < width) {
                int idx = ((y2 - t) * width + x) * 3;
                buffer[idx] = r;     // R
                buffer[idx + 1] = g; // G
                buffer[idx + 2] = b; // B
            }
        }
    }

    // 绘制垂直线（左右边框）
    for (int t = 0; t < thickness; ++t) {
        for (int y = y1; y <= y2; ++y) {
            // 左边框
            if (x1 + t >= 0 && x1 + t < width && y >= 0 && y < height) {
                int idx = (y * width + (x1 + t)) * 3;
                buffer[idx] = r;     // R
                buffer[idx + 1] = g; // G
                buffer[idx + 2] = b; // B
            }
            // 右边框
            if (x2 - t >= 0 && x2 - t < width && y >= 0 && y < height) {
                int idx = (y * width + (x2 - t)) * 3;
                buffer[idx] = r;     // R
                buffer[idx + 1] = g; // G
                buffer[idx + 2] = b; // B
            }
        }
    }
}

// 在RGB888图像上绘制关键点
static void draw_keypoint_rgb888(uint8_t *buffer, int width, int height, int x, int y, uint8_t r, uint8_t g, uint8_t b, int radius) {
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                if (dx * dx + dy * dy <= radius * radius) { // 圆形关键点
                    int idx = (ny * width + nx) * 3;
                    buffer[idx] = r;     // R
                    buffer[idx + 1] = g; // G
                    buffer[idx + 2] = b; // B
                }
            }
        }
    }
}
// 检测任务
static void detect_task(void *arg)
{
    COCOPose *pose_model = new COCOPose();
    frame_buffer_t frame;

    while (1) {
        if (xQueueReceive(detect_queue, &frame, portMAX_DELAY) == pdTRUE) {
            if (!detection_enabled) {
                continue;
            }

            dl::image::img_t img = {.data = frame.buffer,
                                    .width = (uint16_t)frame.width,
                                    .height = (uint16_t)frame.height,
                                    .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888};

            // 运行姿态检测
            auto &pose_results = pose_model->run(img);

            // 更新检测状态
            has_detection = !pose_results.empty();
            if (has_detection) {
                last_detection_time = esp_timer_get_time() / 1000; // 转换为毫秒
            }

            // 根据宏定义控制边框绘制
#if SHOW_BBOX_ONLY_ON_DETECTION
            // 只在检测到人体时绘制边框
            if (has_detection) {
#else
            // 总是尝试绘制边框（如果有检测结果）
            if (true) {
#endif
                if (!pose_results.empty()) {
                    for (const auto &res : pose_results) {
                        // 绘制边界框 - 红色
                        draw_rectangle_rgb888(frame.buffer, frame.width, frame.height,
                                            res.box[0], res.box[1], res.box[2], res.box[3],
                                            255, 0, 0, 2); // 红色，2像素粗细
                        
                                            // 绘制关键点 - 绿色圆点
                    for (int i = 0; i < 17; ++i) {
                        int x = res.keypoint[2 * i];
                        int y = res.keypoint[2 * i + 1];
                        if (x > 0 && y > 0) { // 只绘制有效的关键点
                            draw_keypoint_rgb888(frame.buffer, frame.width, frame.height,
                                               x, y, 0, 255, 0, 3); // 绿色圆点，半径3
                        }
                    }

#if DRAW_SKELETON_LINES
                    // 绘制骨骼连接线 - 蓝色线条
                    for (int i = 0; i < num_skeleton_connections; ++i) {
                        int idx1 = skeleton_connections[i][0];
                        int idx2 = skeleton_connections[i][1];
                        
                        int x1 = res.keypoint[2 * idx1];
                        int y1 = res.keypoint[2 * idx1 + 1];
                        int x2 = res.keypoint[2 * idx2];
                        int y2 = res.keypoint[2 * idx2 + 1];
                        
                        // 只在两个关键点都有效时绘制连接线
                        if (x1 > 0 && y1 > 0 && x2 > 0 && y2 > 0) {
                            draw_line_rgb888(frame.buffer, frame.width, frame.height,
                                           x1, y1, x2, y2, 0, 100, 255, 2); // 蓝色线条，2像素粗细
                        }
                    }
#endif
                    }

#if FREEZE_ON_DETECTION
                    // 如果开启冻结模式，保存当前帧到冻结缓冲区
                    if (frozen_frame_buffer == NULL) {
                        frozen_frame_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
                    }
                    if (frozen_frame_buffer != NULL) {
                        memcpy(frozen_frame_buffer, frame.buffer, FRAME_BUFFER_SIZE);
                        freeze_display = true;
                    }
#endif
                }
            }

            // 准备检测结果
            detection_result_t result = {};
            result.person_count = 0;
            result.confidence = 0.0f;
            result.detected = false;
            strcpy(result.status_text, "");
            
            if (!pose_results.empty()) {
                result.detected = true;
                result.person_count = (int)pose_results.size();
                result.confidence = pose_results.front().score;
                snprintf(result.status_text, sizeof(result.status_text), 
                        "Detected: %d person(s), Conf: %.2f", 
                        result.person_count, result.confidence);
                
                ESP_LOGI(TAG, "检测到 %d 个人", result.person_count);
                for (const auto &res : pose_results) {
                    ESP_LOGI(TAG,
                            "边界框: [%d, %d, %d, %d], 置信度: %.2f",
                            res.box[0], res.box[1], res.box[2], res.box[3], res.score);

                    // 输出关键点位置
                    char log_buf[512];
                    char *p = log_buf;
                    for (int i = 0; i < 17; ++i) {
                        p += sprintf(p, "%s: [%d, %d] ", kpt_names[i], res.keypoint[2 * i], res.keypoint[2 * i + 1]);
                    }
                    ESP_LOGI(TAG, "关键点位置: %s", log_buf);
                }
            } else {
                result.detected = false;
                result.person_count = 0;
                result.confidence = 0.0f;
                snprintf(result.status_text, sizeof(result.status_text), "No person detected");
            }
            
            // 发送结果到结果队列
            xQueueOverwrite(result_queue, &result);
        }
    }

    delete pose_model;
    vTaskDelete(NULL);
}

// UI更新任务
static void ui_update_task(void *arg)
{
    detection_result_t result;
    
    while (1) {
        // 等待检测结果
        if (xQueueReceive(result_queue, &result, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (ksdiy_lvgl_lock(10)) {
                if (status_label != NULL) {
                    lv_label_set_text(status_label, result.status_text);
                }
                
                // 更新详细结果标签
                if (result_label != NULL) {
                    char detail_text[128];
                    if (result.detected) {
                        snprintf(detail_text, sizeof(detail_text), 
                                "Status: ACTIVE\nPersons: %d\nConfidence: %.1f%%", 
                                result.person_count, result.confidence * 100);
                    } else {
                        snprintf(detail_text, sizeof(detail_text), 
                                "Status: SEARCHING\nPersons: 0\nConfidence: --");
                    }
                    lv_label_set_text(result_label, detail_text);
                }
                ksdiy_lvgl_unlock();
            }
        }
    }
}

// 显示任务
static void display_task(void *arg)
{
    frame_buffer_t frame;
    uint32_t current_time;

    while (1) {
        // 等待新的显示帧
        if (xQueueReceive(display_queue, &frame, pdMS_TO_TICKS(33)) == pdTRUE) {
            current_time = esp_timer_get_time() / 1000; // 转换为毫秒
            
            bool should_update_display = true;
            
#if !CONTINUOUS_REFRESH_MODE
            // 非连续刷新模式：检测到人体后暂停刷新
            if (has_detection) {
                should_update_display = false;
            }
#endif

#if FREEZE_ON_DETECTION
            // 冻结模式：检测到人体后显示冻结帧
            if (freeze_display) {
                // 检查是否超过冻结时间
                if (current_time - last_detection_time > DETECTION_HOLD_TIME_MS) {
                    freeze_display = false;
                    has_detection = false;
                }
                
                // 显示冻结的帧（带边框）
                if (frozen_frame_buffer != NULL && ksdiy_lvgl_lock(10)) {
                    camera_img_desc.data = frozen_frame_buffer;
                    lv_img_set_src(camera_img, &camera_img_desc);
                    ksdiy_lvgl_unlock();
                }
                continue; // 跳过正常帧更新
            }
#endif

            // 正常显示逻辑
            if (should_update_display && ksdiy_lvgl_lock(10)) {
                camera_img_desc.data = frame.buffer;
                lv_img_set_src(camera_img, &camera_img_desc);
                ksdiy_lvgl_unlock();
            }
        }
    }
}

// 摄像头采集任务
static void camera_task(void *arg)
{
    uint8_t *camera_buffer = NULL;
    size_t camera_size = 0;
    uint32_t camera_format = 0;
    uint32_t camera_width, camera_height;

    ESP_ERROR_CHECK(ksdiy_camera_get_resolution(&camera_width, &camera_height));
    ESP_LOGI(TAG, "摄像头分辨率: %ldx%ld", camera_width, camera_height);

    // 创建目标图像缓冲区
    uint8_t *resized_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *display_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);

    if (resized_buffer == NULL || display_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        esp_err_t ret = ksdiy_camera_get_frame(&camera_buffer, &camera_size, &camera_format);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        frame_count++;

        // 如果摄像头分辨率已经是目标分辨率，直接使用
        if (camera_width == TARGET_WIDTH && camera_height == TARGET_HEIGHT) {
            memcpy(display_buffer, camera_buffer, FRAME_BUFFER_SIZE);
            memcpy(resized_buffer, camera_buffer, FRAME_BUFFER_SIZE);
        } else {
            // 需要缩放
            dl::image::img_t src_img = {.data = camera_buffer,
                                        .width = (uint16_t)camera_width,
                                        .height = (uint16_t)camera_height,
                                        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888};

            dl::image::img_t dst_img = {.data = resized_buffer,
                                        .width = TARGET_WIDTH,
                                        .height = TARGET_HEIGHT,
                                        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888};

            // 执行图像缩放
            dl::image::resize(src_img, dst_img, dl::image::DL_IMAGE_INTERPOLATE_BILINEAR);
            memcpy(display_buffer, resized_buffer, FRAME_BUFFER_SIZE);
        }

        // 准备显示帧
        frame_buffer_t display_frame = {.buffer = display_buffer,
                                        .width = TARGET_WIDTH,
                                        .height = TARGET_HEIGHT,
                                        .size = FRAME_BUFFER_SIZE,
                                        .format = camera_format};

        // 发送帧到显示队列
        xQueueOverwrite(display_queue, &display_frame);

        // 每20帧发送一次到检测队列
        if (frame_count % 5 == 0) {
            frame_buffer_t detect_frame = {.buffer = resized_buffer,
                                           .width = TARGET_WIDTH,
                                           .height = TARGET_HEIGHT,
                                           .size = FRAME_BUFFER_SIZE,
                                           .format = camera_format};
            xQueueOverwrite(detect_queue, &detect_frame);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // ~100fps采集率
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 姿态检测系统启动");

    // 初始化摄像头
    ESP_ERROR_CHECK(ksdiy_camera_init(KSDIY_VIDEO_FMT_RGB888));

    // 初始化LVGL
    ksdiy_lvgl_port_init();

    // 获取摄像头分辨率
    uint32_t width, height;
    ESP_ERROR_CHECK(ksdiy_camera_get_resolution(&width, &height));
    ESP_LOGI(TAG, "摄像头分辨率: %ldx%ld", width, height);
    // 配置图像描述符（使用目标分辨率）
    camera_img_desc.header.cf = LV_COLOR_FORMAT_RGB888;
    camera_img_desc.header.w = TARGET_WIDTH;
    camera_img_desc.header.h = TARGET_HEIGHT;
    camera_img_desc.data_size = FRAME_BUFFER_SIZE;
    camera_img_desc.data = NULL;
    
    if (ksdiy_lvgl_lock(10)) {
        // 创建主容器
        lv_obj_t *main_screen = lv_scr_act();
        
        // 创建图像显示区域 - 居中显示在屏幕上半部分
        camera_img = lv_img_create(main_screen);
        // lv_obj_center(camera_img);
        
        lv_obj_set_pos(camera_img, 50, 50);  // 图像下方位置
        // lv_obj_set_y(camera_img, 50); // 向上移动，留出下方空间
        
        // 创建状态标签 - 显示在图像下方
        status_label = lv_label_create(main_screen);
        lv_obj_set_pos(status_label, 10, 400);  // 图像下方位置
        lv_obj_set_size(status_label, 460, 30);
        lv_label_set_text(status_label, "Pose Detection System");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0x000000), 0); // 黑色字体
        lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, 0);
        
        // 创建详细结果标签 - 显示在状态标签下方
        result_label = lv_label_create(main_screen);
        lv_obj_set_pos(result_label, 10, 440);
        lv_obj_set_size(result_label, 460, 80);
        lv_label_set_text(result_label, "Status: INITIALIZING\nPersons: --\nConfidence: --");
        lv_obj_set_style_text_color(result_label, lv_color_hex(0x000000), 0); // 黑色字体
        lv_obj_set_style_text_font(result_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(result_label, LV_TEXT_ALIGN_CENTER, 0);
        
        // 创建系统信息标签 - 显示在最下方
        lv_obj_t *info_label = lv_label_create(main_screen);
        lv_obj_set_pos(info_label, 10, 530);
        lv_obj_set_size(info_label, 460, 60);
        char info_text[128];
        snprintf(info_text, sizeof(info_text), 
                "Camera: %ldx%ld | Target: %dx%d | Rate: Every 5 frames", 
                width, height, TARGET_WIDTH, TARGET_HEIGHT);
        lv_label_set_text(info_label, info_text);
        lv_obj_set_style_text_color(info_label, lv_color_hex(0x666666), 0); // 深灰色字体
        lv_obj_set_style_text_font(info_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(info_label, LV_TEXT_ALIGN_CENTER, 0);

        ksdiy_lvgl_unlock();
    }
    // 创建队列
    display_queue = xQueueCreate(1, sizeof(frame_buffer_t));
    detect_queue = xQueueCreate(1, sizeof(frame_buffer_t));
    result_queue = xQueueCreate(1, sizeof(detection_result_t));

    // 创建任务
    xTaskCreate(camera_task, "camera", 8192, NULL, 5, NULL);
    xTaskCreate(display_task, "display", 4096, NULL, 4, NULL);
    xTaskCreate(detect_task, "detect", 16384, NULL, 3, NULL);
    xTaskCreate(ui_update_task, "ui_update", 4096, NULL, 2, NULL);
}
