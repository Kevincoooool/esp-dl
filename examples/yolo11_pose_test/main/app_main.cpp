#include "app_video.h"
#include "camera_test.h"
#include "coco_pose.hpp"
#include "dl_image.hpp"
#include "dl_image_draw.hpp"
#include "dl_image_process.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "ksdiy_lvgl_port.h"
#include <inttypes.h>
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

extern const uint8_t bus_jpg_start[] asm("_binary_bus_jpg_start");
extern const uint8_t bus_jpg_end[] asm("_binary_bus_jpg_end");
const char *TAG = "yolo11n-pose-test";

// 定义骨架连接关系
const int skeleton_pairs[][2] = {
    {5, 6},   // 左右肩
    {5, 7},   // 左肩-左肘
    {7, 9},   // 左肘-左腕
    {6, 8},   // 右肩-右肘
    {8, 10},  // 右肘-右腕
    {5, 11},  // 左肩-左髋
    {6, 12},  // 右肩-右髋
    {11, 12}, // 左右髋
    {11, 13}, // 左髋-左膝
    {13, 15}, // 左膝-左踝
    {12, 14}, // 右髋-右膝
    {14, 16}  // 右膝-右踝
};

// 定义颜色
const std::vector<uint8_t> point_color = {255, 0, 0}; // 红色
const std::vector<uint8_t> line_color = {0, 255, 0};  // 绿色
const std::vector<uint8_t> box_color = {0, 0, 255};   // 蓝色

// 定义目标分辨率
#define TARGET_WIDTH 480
#define TARGET_HEIGHT 480

// 帧缓冲结构体
typedef struct {
    uint8_t *buffer;
    size_t size;
    uint32_t format;
    uint16_t width;
    uint16_t height;
    bool need_free; // 标记是否需要释放内存
} frame_buffer_t;

// 检测结果结构体
typedef struct {
    std::list<dl::detect::result_t> results;
    frame_buffer_t frame;
} detect_result_t;

// 全局队列句柄
static QueueHandle_t detect_queue = NULL;  // 用于发送帧到检测任务
static QueueHandle_t display_queue = NULL; // 用于发送帧到显示任务
static QueueHandle_t result_queue = NULL;  // 用于发送检测结果

// 全局互斥锁
static SemaphoreHandle_t frame_mutex = NULL;

// 关键点名称
const char *kpt_names[17] = {"nose",
                             "left_eye",
                             "right_eye",
                             "left_ear",
                             "right_ear",
                             "left_shoulder",
                             "right_shoulder",
                             "left_elbow",
                             "right_elbow",
                             "left_wrist",
                             "right_wrist",
                             "left_hip",
                             "right_hip",
                             "left_knee",
                             "right_knee",
                             "left_ankle",
                             "right_ankle"};

// 帧率统计结构体
typedef struct {
    uint32_t frame_count;
    int64_t last_time;
    float fps;
} fps_stats_t;

// 更新帧率
static void update_fps(fps_stats_t *stats)
{
    stats->frame_count++;
    int64_t current_time = esp_timer_get_time();
    if (current_time - stats->last_time > 1000000) { // 每秒更新一次
        stats->fps = stats->frame_count * 1000000.0f / (current_time - stats->last_time);
        stats->frame_count = 0;
        stats->last_time = current_time;
    }
}

// 绘制线段的辅助函数
static void draw_line(const dl::image::img_t &img,
                      int x1,
                      int y1,
                      int x2,
                      int y2,
                      const std::vector<uint8_t> &color,
                      uint8_t thickness = 1)
{
    // 使用Bresenham算法绘制线段
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // 在当前点绘制一个圆点
        dl::image::draw_point(img, x1, y1, color, thickness);

        if (x1 == x2 && y1 == y2)
            break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

#define FRAME_BUFFER_SIZE (TARGET_WIDTH * TARGET_HEIGHT * 3)

// 摄像头采集任务
static void camera_task(void *arg)
{
    uint8_t *jpeg_buffer = NULL;
    size_t jpeg_size = 0;
    uint32_t frame_format = 0;
    uint16_t width, height;
    uint32_t frame_count = 0;
    fps_stats_t camera_fps = {0, esp_timer_get_time(), 0};
    
    ESP_ERROR_CHECK(ksdiy_camera_get_resolution((uint32_t *)&width, (uint32_t *)&height));
    
    // 创建目标图像缓冲区
    uint8_t *resized_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *display_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (resized_buffer == NULL || display_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        esp_err_t ret = ksdiy_camera_get_frame(&jpeg_buffer, &jpeg_size, &frame_format);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 创建源图像对象
        dl::image::img_t src_img = {
            .data = jpeg_buffer,
            .width = width,
            .height = height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888
        };

        // 创建目标图像对象
        dl::image::img_t dst_img = {
            .data = resized_buffer,
            .width = TARGET_WIDTH,
            .height = TARGET_HEIGHT,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888
        };

        // 执行图像缩放
        float scale_x, scale_y;
        dl::image::resize(src_img, 
                         dst_img, 
                         dl::image::DL_IMAGE_INTERPOLATE_BILINEAR,
                         0,         // caps
                         nullptr,   // norm_lut
                         {},        // crop_area
                         &scale_x,  // 返回实际X轴缩放比例
                         &scale_y); // 返回实际Y轴缩放比例

        // 复制一份用于显示
        memcpy(display_buffer, resized_buffer, FRAME_BUFFER_SIZE);

        // 创建帧结构体
        frame_buffer_t detect_frame = {
            .buffer = resized_buffer,
            .size = FRAME_BUFFER_SIZE,
            .format = frame_format,
            .width = TARGET_WIDTH,
            .height = TARGET_HEIGHT,
            .need_free = false
        };

        // 每10帧发送一次到检测队列
        if (frame_count % 10 == 0) {
            if (xQueueSend(detect_queue, &detect_frame, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Detect queue full");
            }
        }
        frame_count++;

        // 更新并打印摄像头帧率
        update_fps(&camera_fps);
        if (camera_fps.frame_count == 0) {
            ESP_LOGI(TAG, "Camera FPS: %.2f", camera_fps.fps);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 清理资源
    heap_caps_free(resized_buffer);
    heap_caps_free(display_buffer);
}

// 检测任务
static void detect_task(void *arg)
{
    COCOPose *pose_model = new COCOPose();
    frame_buffer_t frame;
    fps_stats_t detect_fps = {0, esp_timer_get_time(), 0};
    
    while (1) {
        if (xQueueReceive(detect_queue, &frame, portMAX_DELAY) == pdTRUE) {
            dl::image::img_t img = {
                .data = frame.buffer,
                .width = frame.width,
                .height = frame.height,
                .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888
            };

            // 运行姿态检测
            auto &pose_results = pose_model->run(img);

            // 更新并打印检测帧率
            update_fps(&detect_fps);
            if (detect_fps.frame_count == 0) {
                ESP_LOGI(TAG, "Detection FPS: %.2f", detect_fps.fps);
            }

            // 输出检测结果和关键点位置
            if (!pose_results.empty()) {
                ESP_LOGI(TAG, "检测到 %d 个人", pose_results.size());
                for (const auto &res : pose_results) {
                    ESP_LOGI(TAG,
                            "边界框: [%d, %d, %d, %d], 置信度: %.2f",
                            res.box[0],
                            res.box[1],
                            res.box[2],
                            res.box[3],
                            res.score);

                    // 输出关键点位置
                    char log_buf[512];
                    char *p = log_buf;
                    for (int i = 0; i < 17; ++i) {
                        p += sprintf(p, "%s: [%d, %d] ", 
                                   kpt_names[i], 
                                   res.keypoint[2 * i], 
                                   res.keypoint[2 * i + 1]);
                    }
                    ESP_LOGI(TAG, "关键点位置: %s", log_buf);

                    // 创建检测结果并发送到结果队列
                    detect_result_t result;
                    result.results.push_back(res);
                    result.frame = frame;
                    if (xQueueSend(result_queue, &result, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Result queue full");
                    }
                }
            } else {
                // 如果没有检测到人，也发送原始帧到显示队列
                if (xQueueSend(display_queue, &frame, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Display queue full");
                }
            }
        }
    }
}

// 显示任务
static void display_task(void *arg)
{
    // 初始化LVGL
    ksdiy_lvgl_port_init();

    // 创建图像显示对象
    lv_obj_t *img_obj = lv_img_create(lv_scr_act());
    lv_obj_center(img_obj);

    // 创建图像描述符
    lv_img_dsc_t img_dsc;
    detect_result_t result;
    
    while (1) {
        // 尝试接收检测结果(非阻塞)
        if (xQueueReceive(result_queue, &result, 0) == pdTRUE) {
            // 创建图像对象用于绘制
            dl::image::img_t img = {
                .data = result.frame.buffer,
                .width = result.frame.width,
                .height = result.frame.height,
                .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888
            };

            // 绘制每个检测到的人的边界框和关键点
            for (const auto &res : result.results) {
                // 验证并修正边界框坐标
                int x1 = res.box[0];
                int y1 = res.box[1];
                int x2 = res.box[2];
                int y2 = res.box[3];

                // 确保坐标在有效范围内
                x1 = std::max(0, std::min(x1, img.width - 1));
                y1 = std::max(0, std::min(y1, img.height - 1));
                x2 = std::max(0, std::min(x2, img.width - 1));
                y2 = std::max(0, std::min(y2, img.height - 1));

                // 确保 x2 > x1 且 y2 > y1
                if (x2 <= x1) {
                    std::swap(x1, x2);
                    x2 = std::min(x2 + 1, img.width - 1);  // 确保宽度至少为1
                }
                if (y2 <= y1) {
                    std::swap(y1, y2);
                    y2 = std::min(y2 + 1, img.height - 1); // 确保高度至少为1
                }

                // 绘制边界框
                dl::image::draw_hollow_rectangle(img,
                    x1, y1,     // 左上角坐标
                    x2, y2,     // 右下角坐标
                    box_color,  // 使用蓝色
                    2);        // 线宽为2像素

                // 绘制关键点
                for (int i = 0; i < 17; i++) {
                    int x = res.keypoint[2 * i];
                    int y = res.keypoint[2 * i + 1];
                    
                    // 验证关键点坐标
                    if (x > 0 && x < img.width && y > 0 && y < img.height) {
                        dl::image::draw_point(img, x, y,
                            point_color,  // 使用红色
                            3);          // 点的半径为3像素
                    }
                }

                // 绘制骨架连接线
                for (const auto &pair : skeleton_pairs) {
                    int start_idx = pair[0];
                    int end_idx = pair[1];
                    
                    // 获取起点和终点坐标
                    int start_x = res.keypoint[2 * start_idx];
                    int start_y = res.keypoint[2 * start_idx + 1];
                    int end_x = res.keypoint[2 * end_idx];
                    int end_y = res.keypoint[2 * end_idx + 1];

                    // 验证线段端点坐标
                    if (start_x > 0 && start_x < img.width && 
                        start_y > 0 && start_y < img.height &&
                        end_x > 0 && end_x < img.width && 
                        end_y > 0 && end_y < img.height) {
                        draw_line(img, start_x, start_y, end_x, end_y, line_color, 2);
                    }
                }
            }

            // 更新显示
            if (ksdiy_lvgl_lock(10)) {
                // 更新图像描述符
                img_dsc.header.w = result.frame.width;
                img_dsc.header.h = result.frame.height;
                img_dsc.header.cf = LV_COLOR_FORMAT_RGB888;
                img_dsc.data_size = result.frame.size;
                img_dsc.data = (uint8_t *)img.data;

                // 更新显示
                lv_img_set_src(img_obj, &img_dsc);
                lv_task_handler();
                
                ksdiy_lvgl_unlock();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
#if CONFIG_COCO_POSE_MODEL_IN_SDCARD
    ESP_ERROR_CHECK(bsp_sdcard_mount());
#endif

    ESP_LOGI(TAG, "🚀 ESP32-P4 YOLO11n-pose Performance Test Started");

    // 初始化摄像头
    esp_err_t ret = ksdiy_camera_init(KSDIY_VIDEO_FMT_RGB888);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return;
    }

    // 创建队列
    detect_queue = xQueueCreate(2, sizeof(frame_buffer_t));
    display_queue = xQueueCreate(10, sizeof(frame_buffer_t));
    result_queue = xQueueCreate(2, sizeof(detect_result_t));

    // 创建互斥锁
    frame_mutex = xSemaphoreCreateMutex();

    // 创建任务
    xTaskCreatePinnedToCore(camera_task, "camera_task", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(detect_task, "detect_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(display_task, "display_task", 8192, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "All tasks created successfully");

    // 主循环
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
