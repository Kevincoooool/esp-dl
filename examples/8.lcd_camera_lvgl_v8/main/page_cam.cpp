/*
 * @Descripttion : 带坐姿检测的摄像头页面
 * @version      : v2.0
 * @Author       : Kevincoooool + ESP-DL Integration
 * @Date         : 2021-06-05 10:13:51
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-01-01 12:00:00
 * @FilePath: \examples\8.lcd_camera_lvgl_v8\main\page_cam.cpp
 */
#include "page_cam.h"
#include "posture_analyzer.hpp"
#include "app_camera.h"
#include "ksdiy_lvgl_port.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "page_cam";

// Global variables
ui_page_cam_t ui_page_cam;
static PostureAnalyzer* posture_analyzer = nullptr;
static bool detection_enabled = true;
static uint32_t frame_count = 0;
static uint32_t detection_count = 0;

// Function declarations
extern "C" {
    void Cam_Task(void *pvParameters);
    void process_posture_detection(camera_fb_t *fb_data);
    esp_err_t init_posture_detection(void);
    void page_cam_load(void);
    void page_cam_end(void);
}

// Detection toggle callback
static void detection_toggle_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *obj = (lv_obj_t*)lv_event_get_target(e);
        detection_enabled = lv_obj_has_state(obj, LV_STATE_CHECKED);
        ESP_LOGI(TAG, "AI Detection %s", detection_enabled ? "enabled" : "disabled");
    }
}

esp_err_t init_posture_detection(void) {
    ESP_LOGI(TAG, "Initializing real-time YOLO11n-pose posture detection system...");
    
    if (posture_analyzer) {
        delete posture_analyzer;
    }
    
    posture_analyzer = new PostureAnalyzer();
    if (!posture_analyzer) {
        ESP_LOGE(TAG, "Failed to create PostureAnalyzer");
        return ESP_FAIL;
    }
    
    esp_err_t ret = posture_analyzer->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PostureAnalyzer: %s", esp_err_to_name(ret));
        delete posture_analyzer;
        posture_analyzer = nullptr;
        return ret;
    }
    
    // Set detection thresholds
    posture_analyzer->set_thresholds(
        20.0f,  // Head tilt angle threshold (degrees)
        0.7f,   // Lying head Y position threshold
        25.0f,  // Hunch angle threshold (degrees)
        0.4f    // Minimum detection confidence
    );
    
    ESP_LOGI(TAG, "Real-time YOLO11n-pose posture detection system initialized successfully");
    return ESP_OK;
}

void process_posture_detection(camera_fb_t *fb_data) {
    if (!posture_analyzer || !detection_enabled || !fb_data) {
        return;
    }
    
    frame_count++;
    
    // Process every 20 frames for maximum performance (reduce AI workload)
    if (frame_count % 20 != 0) {
        return;
    }
    
    detection_count++;
    
    // 记录开始时间
    uint32_t start_time = esp_timer_get_time() / 1000;  // 转换为毫秒
    
    // Run optimized YOLO11n-pose detection  
    ESP_LOGD(TAG, "Starting AI inference...");
    PostureResult result = posture_analyzer->analyze_frame(fb_data);
    ESP_LOGD(TAG, "AI inference completed");
    
    // 计算推理时间
    uint32_t inference_time = (esp_timer_get_time() / 1000) - start_time;
    
    // Update UI display
    if (ksdiy_lvgl_lock(100)) {
        // Update posture status text
        const char* state_text = posture_analyzer->get_state_description(result.state);
        lv_label_set_text(ui_page_cam.posture_status_label, state_text);
        
        // Update posture icon
        const char* state_icon = posture_analyzer->get_state_icon(result.state);
        lv_label_set_text(ui_page_cam.posture_icon_label, state_icon);
        
        // Update performance info display
        char info_text[128];
        if (result.state != PostureState::UNKNOWN) {
            snprintf(info_text, sizeof(info_text), 
                     "推理: %lums | per: %.0f%% | FPS: %.1f",
                     inference_time,
                     result.confidence * 100,
                     1000.0f / inference_time);
        } else {
            snprintf(info_text, sizeof(info_text), "AI fenxi zhong...");
        }
        lv_label_set_text(ui_page_cam.confidence_label, info_text);
        
        // Set status colors
        switch (result.state) {
            case PostureState::NORMAL_SITTING:
                lv_obj_set_style_text_color(ui_page_cam.posture_status_label, lv_color_hex(0x00AA00), 0);
                break;
            case PostureState::LEANING_FORWARD:
                lv_obj_set_style_text_color(ui_page_cam.posture_status_label, lv_color_hex(0xFFAA00), 0);
                break;
            case PostureState::LYING_ON_TABLE:
            case PostureState::HEAD_TILTED:
            case PostureState::HUNCHED_BACK:
                lv_obj_set_style_text_color(ui_page_cam.posture_status_label, lv_color_hex(0xFF4400), 0);
                break;
            case PostureState::UNKNOWN:
            default:
                lv_obj_set_style_text_color(ui_page_cam.posture_status_label, lv_color_hex(0x888888), 0);
                break;
        }
        
        ksdiy_lvgl_unlock();
    }
    
    // Log performance and detection results
    if (detection_count % 5 == 0) {  // Log every 5 detections for performance monitoring
        ESP_LOGI(TAG, "🚀 AI性能 [%lu/%lu] - 推理耗时: %lums, 状态: %s, 置信度: %.1f%%, FPS: %.1f", 
                 detection_count, frame_count,
                 inference_time,
                 posture_analyzer->get_state_description(result.state),
                 result.confidence * 100,
                 1000.0f / inference_time);
    }
}

void Cam_Task(void *pvParameters) {
    ESP_LOGI(TAG, "Camera task started with real-time YOLO11n-pose detection");
    
    uint32_t log_counter = 0;
    
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Log camera frame info every 100 frames for performance
        if (log_counter % 100 == 0) {
            ESP_LOGI(TAG, "Camera frame: %dx%d, format: %d, size: %zu bytes", 
                     fb->width, fb->height, fb->format, fb->len);
        }
        log_counter++;
        
        // Process real-time posture detection
        process_posture_detection(fb);
        
        // Display camera image to LCD (optional)
        if (ksdiy_lvgl_lock(100)) {
            // Image display logic can be added here
            // Currently prioritize AI detection performance
            ksdiy_lvgl_unlock();
        }
        
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(200));  // 5fps for maximum AI performance
    }
}

void page_cam_load(void) {
    ESP_LOGI(TAG, "Loading camera page with real-time YOLO11n-pose detection");
    
    // Create main container
    lv_obj_t *page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES);
    lv_obj_center(page);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(page, lv_color_hex(0x202020), 0);
    
    // Create title (simplified)
    ui_page_cam.title = lv_label_create(page);
    lv_label_set_text(ui_page_cam.title, "AI spd");
    lv_obj_set_style_text_font(ui_page_cam.title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(ui_page_cam.title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(ui_page_cam.title, LV_ALIGN_TOP_MID, 0, 5);
    
    // Create main result display (large and centered)
    ui_page_cam.posture_icon_label = lv_label_create(page);
    lv_label_set_text(ui_page_cam.posture_icon_label, "❓");
    lv_obj_set_style_text_font(ui_page_cam.posture_icon_label, &lv_font_montserrat_28, 0);
    lv_obj_align(ui_page_cam.posture_icon_label, LV_ALIGN_CENTER, 0, -40);
    
    // Large status text
    ui_page_cam.posture_status_label = lv_label_create(page);
    lv_label_set_text(ui_page_cam.posture_status_label, "init...");
    lv_obj_set_style_text_font(ui_page_cam.posture_status_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(ui_page_cam.posture_status_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(ui_page_cam.posture_status_label, LV_ALIGN_CENTER, 0, 0);
    
    // Performance info (simplified)
    ui_page_cam.confidence_label = lv_label_create(page);
    lv_label_set_text(ui_page_cam.confidence_label, "AI init...");
    lv_obj_set_style_text_font(ui_page_cam.confidence_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_page_cam.confidence_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(ui_page_cam.confidence_label, LV_ALIGN_CENTER, 0, 40);
    
    // Detection toggle (moved to bottom)
    ui_page_cam.detection_toggle = lv_switch_create(page);
    lv_obj_align(ui_page_cam.detection_toggle, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_state(ui_page_cam.detection_toggle, LV_STATE_CHECKED);
    lv_obj_add_event_cb(ui_page_cam.detection_toggle, detection_toggle_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Initialize real-time posture detection
    esp_err_t ret = init_posture_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize optimized posture detection");
        lv_label_set_text(ui_page_cam.posture_status_label, "AI初始化失败");
        lv_obj_set_style_text_color(ui_page_cam.posture_status_label, lv_color_hex(0xFF0000), 0);
        return;
    }
    
    // Initialize camera  
    ESP_LOGI(TAG, "Initializing camera with JPEG format for YOLO11n-pose...");
    app_camera_init();
    ESP_LOGI(TAG, "Camera initialized successfully");
    
    // Start camera task with optimized settings
    ESP_LOGI(TAG, "Starting optimized YOLO11n-pose detection task...");
    xTaskCreatePinnedToCore(Cam_Task, "yolo11_cam_task", 32768, NULL, 3, NULL, 1);  // Increased stack, lower priority
    
    ESP_LOGI(TAG, "✅ 高性能AI坐姿识别系统加载完成");
}

void page_cam_end(void) {
    ESP_LOGI(TAG, "Ending real-time camera page");
    
    // Clean up posture analyzer
    if (posture_analyzer) {
        delete posture_analyzer;
        posture_analyzer = nullptr;
    }
    
    ESP_LOGI(TAG, "Real-time camera page ended");
} 