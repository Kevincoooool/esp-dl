#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化LVGL
 */
void ksdiy_lvgl_port_init(void);

/**
 * @brief 获取LVGL互斥锁
 * 
 * @param timeout_ms 超时时间（毫秒）
 * @return true 成功获取锁
 * @return false 获取锁失败
 */
bool ksdiy_lvgl_lock(uint32_t timeout_ms);

/**
 * @brief 释放LVGL互斥锁
 */
void ksdiy_lvgl_unlock(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

// #endif