#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "posture_analyzer.hpp"

// 注意：这里使用基础的姿态检测框架，实际项目中需要集成ESP-DL的模型加载
// 由于ESP-DL的具体模型API可能需要根据实际版本调整，这里提供基础框架

static const char *TAG = "PostureDetection";

// 相机配置（ESP32-S3 + OV2640）
camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = 4,
    .pin_sscb_sda = 18,
    .pin_sscb_scl = 23,
    .pin_d7 = 36,
    .pin_d6 = 37,
    .pin_d5 = 38,
    .pin_d4 = 39,
    .pin_d3 = 35,
    .pin_d2 = 14,
    .pin_d1 = 13,
    .pin_d0 = 34,
    .pin_vsync = 5,
    .pin_href = 27,
    .pin_pclk = 25,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size = FRAMESIZE_QVGA, // 320x240
    .jpeg_quality = 10,
    .fb_count = 1
};

// 示例关键点数据（用于演示，实际应用中来自模型推理）
std::vector<Keypoint> generate_demo_keypoints(int demo_type) {
    std::vector<Keypoint> keypoints(17);
    
    // 设置基础置信度
    for (auto& kp : keypoints) {
        kp.confidence = 0.8;
    }
    
    switch (demo_type % 4) {
        case 0: // 正常坐姿
            keypoints[NOSE] = {160, 80, 0.9};
            keypoints[LEFT_EYE] = {150, 70, 0.8};
            keypoints[RIGHT_EYE] = {170, 70, 0.8};
            keypoints[LEFT_SHOULDER] = {130, 120, 0.9};
            keypoints[RIGHT_SHOULDER] = {190, 120, 0.9};
            break;
            
        case 1: // 趴桌
            keypoints[NOSE] = {160, 150, 0.9};
            keypoints[LEFT_EYE] = {150, 140, 0.8};
            keypoints[RIGHT_EYE] = {170, 140, 0.8};
            keypoints[LEFT_SHOULDER] = {130, 160, 0.9};
            keypoints[RIGHT_SHOULDER] = {190, 160, 0.9};
            break;
            
        case 2: // 歪头
            keypoints[NOSE] = {160, 80, 0.9};
            keypoints[LEFT_EYE] = {145, 85, 0.8};
            keypoints[RIGHT_EYE] = {175, 65, 0.8};
            keypoints[LEFT_SHOULDER] = {130, 120, 0.9};
            keypoints[RIGHT_SHOULDER] = {190, 120, 0.9};
            break;
            
        case 3: // 驼背
            keypoints[NOSE] = {160, 80, 0.9};
            keypoints[LEFT_EYE] = {150, 70, 0.8};
            keypoints[RIGHT_EYE] = {170, 70, 0.8};
            keypoints[LEFT_SHOULDER] = {110, 120, 0.9};
            keypoints[RIGHT_SHOULDER] = {210, 120, 0.9};
            break;
    }
    
    return keypoints;
}

void posture_detection_task(void *arg) {
    PostureAnalyzer analyzer;
    int demo_counter = 0;
    
    // 可调节的检测参数
    analyzer.setHeadTiltThreshold(12.0);   // 头部倾斜阈值
    analyzer.setLyingHeightRatio(0.25);    // 趴桌检测阈值
    
    ESP_LOGI(TAG, "开始坐姿检测...");
    
    while (true) {
        // 获取摄像头图像
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "获取摄像头图像失败");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        ESP_LOGI(TAG, "捕获图像: %dx%d, 大小: %d bytes", 
                 fb->width, fb->height, fb->len);
        
        // 在实际应用中，这里应该进行：
        // 1. 图像预处理
        // 2. 模型推理得到关键点
        // 3. 后处理提取关键点坐标
        
        // 现在使用演示数据
        std::vector<Keypoint> keypoints = generate_demo_keypoints(demo_counter++);
        
        // 分析坐姿
        PostureState state = analyzer.analyzePosture(keypoints);
        const char* description = analyzer.getPostureDescription(state);
        
        // 输出检测结果
        if (state == NORMAL_SITTING) {
            ESP_LOGI(TAG, "✅ 检测结果: %s", description);
        } else {
            ESP_LOGW(TAG, "❌ 检测结果: %s", description);
            
            // 可以添加警告机制
            // 例如：GPIO输出控制LED、蜂鸣器等
        }
        
        // 输出详细信息
        float head_tilt = analyzer.calculateHeadTilt(keypoints);
        ESP_LOGI(TAG, "头部倾斜角度: %.1f°", head_tilt);
        
        // 输出关键点信息（调试用）
        ESP_LOGD(TAG, "鼻子位置: (%.1f, %.1f), 置信度: %.2f", 
                keypoints[NOSE].x, keypoints[NOSE].y, keypoints[NOSE].confidence);
        ESP_LOGD(TAG, "左肩位置: (%.1f, %.1f), 右肩位置: (%.1f, %.1f)", 
                keypoints[LEFT_SHOULDER].x, keypoints[LEFT_SHOULDER].y,
                keypoints[RIGHT_SHOULDER].x, keypoints[RIGHT_SHOULDER].y);
        
        // 释放图像缓冲区
        esp_camera_fb_return(fb);
        
        // 检测间隔
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "ESP-DL 坐姿检测系统启动");
    ESP_LOGI(TAG, "版本: 演示版本 v1.0");
    
    // 初始化相机
    ESP_LOGI(TAG, "初始化摄像头...");
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败: 0x%x", err);
        return;
    }
    ESP_LOGI(TAG, "摄像头初始化成功");
    
    // 在实际应用中，这里应该加载ESP-DL模型
    ESP_LOGI(TAG, "注意: 当前版本使用演示数据");
    ESP_LOGI(TAG, "实际部署时需要:");
    ESP_LOGI(TAG, "1. 加载量化后的YOLO11n-pose模型");
    ESP_LOGI(TAG, "2. 集成ESP-DL模型推理API");
    ESP_LOGI(TAG, "3. 实现图像预处理和后处理");
    
    // 创建坐姿检测任务
    xTaskCreatePinnedToCore(
        posture_detection_task,
        "posture_detect",
        8192,  // 栈大小
        NULL,
        5,     // 优先级
        NULL,
        1      // 运行在核心1
    );
    
    ESP_LOGI(TAG, "坐姿检测任务已启动");
    
    // 主循环 - 系统状态监控
    while (true) {
        // 输出系统状态
        ESP_LOGI(TAG, "系统运行中... 自由堆内存: %d bytes", 
                esp_get_free_heap_size());
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10秒输出一次状态
    }
} 