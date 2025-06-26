/*
 * @Author: Kevincoooool + ESP-DL Integration
 * @Date: 2021-12-06 16:24:39
 * @Description: Real-time Posture Detection Camera Page Header
 * @version: v3.0
 * @Filename: page_cam.h
 * @LastEditTime: 2024-01-01 12:00:00
 * @FilePath: \examples\8.lcd_camera_lvgl_v8\main\page_cam.h
 */

#ifndef _PAGE_CAM_H
#define _PAGE_CAM_H

#include "app_camera.h"
#include "posture_analyzer.hpp"  // Use real YOLO11n-pose analyzer
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    lv_obj_t *img;
    lv_obj_t *title;
    lv_obj_t *posture_status_label;
    lv_obj_t *posture_icon_label;
    lv_obj_t *detection_toggle;
    lv_obj_t *confidence_label;
} ui_page_cam_t;

extern ui_page_cam_t ui_page_cam;

// Function declarations
void Cam_Task(void *pvParameters);
void page_cam_load(void);
void page_cam_end(void);

// Real-time posture detection functions
void process_posture_detection(camera_fb_t *fb_data);

// Initialize real-time posture detection system
esp_err_t init_posture_detection(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _PAGE_CAM_H */


