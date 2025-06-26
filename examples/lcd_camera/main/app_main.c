#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7703.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "ksdiy_lvgl_port.h"
#include "app_video.h"

// Forward declarations for C++ components
typedef struct PostureAnalyzer PostureAnalyzer;

typedef enum {
    POSTURE_NORMAL_SITTING = 0,
    POSTURE_LYING_ON_TABLE = 1,
    POSTURE_HEAD_TILTED = 2,
    POSTURE_HUNCHED_BACK = 3,
    POSTURE_LEANING_FORWARD = 4,
    POSTURE_UNKNOWN = 5
} PostureState;

typedef struct {
    PostureState state;
    float confidence;
    float head_tilt_angle;
    float spine_curve_angle;
    float shoulder_balance;
    bool head_low_position;
} CPostureResult;

extern PostureAnalyzer* posture_analyzer_create(void);
extern void posture_analyzer_destroy(PostureAnalyzer* analyzer);
extern int posture_analyzer_init(PostureAnalyzer* analyzer);
extern CPostureResult posture_analyzer_analyze_frame(PostureAnalyzer* analyzer, uint8_t* buffer, size_t buffer_size, uint32_t width, uint32_t height, uint32_t format);
extern void posture_analyzer_set_thresholds(PostureAnalyzer* analyzer, float head_tilt, float lying_head, float hunch_angle, float min_conf);
extern const char* posture_get_state_description(PostureState state);
extern const char* posture_get_state_icon(PostureState state);

static const char *TAG = "ESP32P4_PostureDetection";

// 定义LVGL界面对象
static lv_obj_t *camera_img;
static lv_img_dsc_t camera_img_desc;
static lv_obj_t *posture_status_label;
static lv_obj_t *posture_icon_label;
static lv_obj_t *confidence_label;
static lv_obj_t *detection_toggle;

// Posture detection related
static PostureAnalyzer* posture_analyzer = NULL;
static bool detection_enabled = true;
static uint32_t frame_count = 0;
static uint32_t detection_count = 0;

// 定义摄像头刷新任务
static void camera_task(void *arg);
static void detection_toggle_cb(lv_event_t *e);
static esp_err_t init_posture_detection(void);
static void process_posture_detection(uint8_t *buffer, size_t buffer_size, uint32_t width, uint32_t height, uint32_t format);

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 Posture Detection System Starting");
    
    // Initialize camera with RGB565 format
    ESP_ERROR_CHECK(ksdiy_camera_init(KSDIY_VIDEO_FMT_RGB565));
    
    // Initialize LVGL
    ksdiy_lvgl_port_init();
    ksdiy_lvgl_lock(-1);
    
    // Get camera resolution
    uint32_t width, height;
    ESP_ERROR_CHECK(ksdiy_camera_get_resolution(&width, &height));
    ESP_LOGI(TAG, "Camera resolution: %ldx%ld", width, height);
    
    // Create full screen container
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_center(cont);
    
    // Create title
    lv_obj_t *title = lv_label_create(cont);
    lv_label_set_text(title, "ESP32-P4 AI Posture Detection");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x0080FF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // Create image object (smaller size to leave space for UI elements)
    camera_img = lv_img_create(cont);
    lv_obj_set_size(camera_img, 640, 480);
    lv_obj_align(camera_img, LV_ALIGN_TOP_MID, 0, 50);
    
    // Set image descriptor
    camera_img_desc.header.w = width;
    camera_img_desc.header.h = height;
    camera_img_desc.header.cf = LV_COLOR_FORMAT_RGB565;
    camera_img_desc.data_size = width * height * 2; // RGB565 2 bytes per pixel
    camera_img_desc.data = NULL; // Will be updated in task
    
    // Create posture status display
    posture_status_label = lv_label_create(cont);
    lv_label_set_text(posture_status_label, "Detecting...");
    lv_obj_set_style_text_font(posture_status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(posture_status_label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(posture_status_label, LV_ALIGN_BOTTOM_LEFT, 20, -80);
    
    // Create icon display
    posture_icon_label = lv_label_create(cont);
    lv_label_set_text(posture_icon_label, "🔍");
    lv_obj_set_style_text_font(posture_icon_label, &lv_font_montserrat_14, 0);
    lv_obj_align(posture_icon_label, LV_ALIGN_BOTTOM_LEFT, 20, -120);
    
    // Create confidence display
    confidence_label = lv_label_create(cont);
    lv_label_set_text(confidence_label, "Confidence: --");
    lv_obj_set_style_text_font(confidence_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(confidence_label, lv_color_hex(0x808080), 0);
    lv_obj_align(confidence_label, LV_ALIGN_BOTTOM_LEFT, 20, -50);
    
    // Create AI detection toggle
    detection_toggle = lv_switch_create(cont);
    lv_obj_align(detection_toggle, LV_ALIGN_BOTTOM_RIGHT, -20, -60);
    lv_obj_add_state(detection_toggle, LV_STATE_CHECKED);
    lv_obj_add_event_cb(detection_toggle, detection_toggle_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Add toggle label
    lv_obj_t *toggle_label = lv_label_create(cont);
    lv_label_set_text(toggle_label, "AI Detection");
    lv_obj_align(toggle_label, LV_ALIGN_BOTTOM_RIGHT, -20, -30);
    
    ksdiy_lvgl_unlock();
    
    // Initialize posture detection
    esp_err_t ret = init_posture_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Posture detection initialization failed");
        ksdiy_lvgl_lock(-1);
        lv_label_set_text(posture_status_label, "AI Init Failed");
        lv_obj_set_style_text_color(posture_status_label, lv_color_hex(0xFF0000), 0);
        ksdiy_lvgl_unlock();
    } else {
        ESP_LOGI(TAG, "✅ Posture detection system initialized successfully");
    }
    
    // Create camera refresh task
    ESP_LOGI(TAG, "Starting camera detection task...");
    xTaskCreate(camera_task, "posture_cam_task", 32768, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "ESP32-P4 AI Posture Detection System Started");
}

// Camera refresh task
static void camera_task(void *arg)
{
    uint8_t *frame_buffer;
    size_t frame_size;
    uint32_t frame_format;
    uint32_t width, height;
    
    // Get camera resolution
    ksdiy_camera_get_resolution(&width, &height);
    
    while (1) {
        // Get one frame
        esp_err_t ret = ksdiy_camera_get_frame(&frame_buffer, &frame_size, &frame_format);
        if (ret == ESP_OK) {
            frame_count++;
            
            // Update image data
            ksdiy_lvgl_lock(-1);
            camera_img_desc.data = frame_buffer;
            lv_img_set_src(camera_img, &camera_img_desc);
            ksdiy_lvgl_unlock();
            
            // Perform posture detection (every 30 frames due to 640x640 resolution)
            if (detection_enabled && frame_count % 30 == 0) {
                process_posture_detection(frame_buffer, frame_size, width, height, frame_format);
            }
        } else {
            ESP_LOGE(TAG, "Failed to get camera frame: %s", esp_err_to_name(ret));
        }
        
        // Delay 30ms, approximately 33fps
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// Detection toggle callback
static void detection_toggle_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *obj = (lv_obj_t*)lv_event_get_target(e);
        detection_enabled = lv_obj_has_state(obj, LV_STATE_CHECKED);
        ESP_LOGI(TAG, "AI Detection %s", detection_enabled ? "Enabled" : "Disabled");
    }
}

// Initialize posture detection
static esp_err_t init_posture_detection(void) {
    ESP_LOGI(TAG, "Initializing ESP32-P4 YOLO11n-pose posture detection system...");
    
    if (posture_analyzer) {
        posture_analyzer_destroy(posture_analyzer);
    }
    
    posture_analyzer = posture_analyzer_create();
    if (!posture_analyzer) {
        ESP_LOGE(TAG, "Failed to create PostureAnalyzer");
        return ESP_FAIL;
    }
    
    int ret = posture_analyzer_init(posture_analyzer);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to initialize PostureAnalyzer");
        posture_analyzer_destroy(posture_analyzer);
        posture_analyzer = NULL;
        return ESP_FAIL;
    }
    
    // Set detection thresholds
    posture_analyzer_set_thresholds(posture_analyzer,
        20.0f,  // Head tilt angle threshold (degrees)
        0.7f,   // Lying head Y position threshold
        25.0f,  // Hunch angle threshold (degrees)  
        0.4f    // Minimum detection confidence
    );
    
    ESP_LOGI(TAG, "ESP32-P4 YOLO11n-pose posture detection system initialized successfully");
    return ESP_OK;
}

// Process posture detection
static void process_posture_detection(uint8_t *buffer, size_t buffer_size, uint32_t width, uint32_t height, uint32_t format) {
    if (!posture_analyzer || !detection_enabled || !buffer) {
        return;
    }
    
    detection_count++;
    
    // Record start time
    uint32_t total_start = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    ESP_LOGI(TAG, "🎯 Starting posture detection %lu: %ldx%ld, format=0x%lx, size=%zu", 
             detection_count, width, height, format, buffer_size);
    
    // Run AI inference
    ESP_LOGD(TAG, "Starting AI inference...");
    CPostureResult result = posture_analyzer_analyze_frame(posture_analyzer, buffer, buffer_size, width, height, format);
    ESP_LOGD(TAG, "AI inference completed");
    
    // Calculate inference time
    uint32_t total_time = (esp_timer_get_time() / 1000) - total_start;
    
    // Get state description for display and logging
    const char* state_desc = posture_get_state_description(result.state);
    
    ESP_LOGI(TAG, "🏁 Detection %lu completed in %lums: %s (%.1f%%)", 
             detection_count, total_time, state_desc, result.confidence * 100);
    
    // Update UI display
    if (ksdiy_lvgl_lock(100)) {
        // Update status text
        lv_label_set_text(posture_status_label, state_desc);
        
        // Update icon
        const char* icon = posture_get_state_icon(result.state);
        lv_label_set_text(posture_icon_label, icon);
        
        // Set color based on state
        uint32_t color = 0x00FF00; // Green
        if (result.state != POSTURE_NORMAL_SITTING && result.state != POSTURE_UNKNOWN) {
            color = 0xFF8000; // Orange
        }
        lv_obj_set_style_text_color(posture_status_label, lv_color_hex(color), 0);
        
        // Update confidence
        char conf_text[64];
        snprintf(conf_text, sizeof(conf_text), "Confidence: %.1f%% (%lums)", 
                result.confidence * 100, total_time);
        lv_label_set_text(confidence_label, conf_text);
        
        ksdiy_lvgl_unlock();
    }
    
    if (result.state != POSTURE_NORMAL_SITTING && result.state != POSTURE_UNKNOWN) {
        ESP_LOGW(TAG, "⚠️ Poor posture detected: %s", state_desc);
        ESP_LOGW(TAG, "   Head tilt: %.1f°, Spine angle: %.1f°", 
                 result.head_tilt_angle, result.spine_curve_angle);
    }
}