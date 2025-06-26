
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "ksdiy_lvgl_port.h"
#include "app_camera.h"
#include "page_cam.h"
#include "lv_demos.h"


extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ksdiy_lvgl_port_init();
    if (ksdiy_lvgl_lock(10))
    {
    //     // example_lvgl_demo_ui(disp);
        // lv_demo_music();
    //     // Release the mutex
        page_cam_load();
        // lv_demo_widgets();
        ksdiy_lvgl_unlock();
    }
}
