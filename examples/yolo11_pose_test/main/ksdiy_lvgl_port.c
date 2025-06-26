#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"

#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lvgl_port.h"

static const char *TAG = "example";

//  #define   LCD_TYPE_ST7703
 #define   LCD_TYPE_GC9503


#ifdef LCD_TYPE_ST7703
#include "esp_lcd_st7703.h"
#define PANEL_BUS_DSI_2CH_CONFIG ST7703_PANEL_BUS_DSI_2CH_CONFIG
#define PANEL_IO_DBI_CONFIG ST7703_PANEL_IO_DBI_CONFIG
#define PANEL_DPI_CONFIG ST7703_720_720_PANEL_60HZ_DPI_CONFIG
#define KSDIY_MIPI_DSI_LCD_H_RES 720
#define KSDIY_MIPI_DSI_LCD_V_RES 720

#elif defined LCD_TYPE_GC9503
#include "esp_lcd_gc9503.h"
#define PANEL_BUS_DSI_2CH_CONFIG GC9503_PANEL_BUS_DSI_2CH_CONFIG
#define PANEL_IO_DBI_CONFIG GC9503_PANEL_IO_DBI_CONFIG
#define PANEL_DPI_CONFIG GC9503_480_800_PANEL_60HZ_DPI_CONFIG
#define KSDIY_MIPI_DSI_LCD_H_RES 480
#define KSDIY_MIPI_DSI_LCD_V_RES 800

#endif

#if LV_COLOR_DEPTH == 16
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)

#define LVGL_PX_FORMAT (LV_COLOR_FORMAT_RGB565)
#define BSP_LCD_COLOR_DEPTH (16)
#elif LV_COLOR_DEPTH >= 24
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB888)
#define LVGL_PX_FORMAT (LV_COLOR_FORMAT_RGB888)
#define BSP_LCD_COLOR_DEPTH (24)
#endif
// “VDD_MIPI_DPHY”应供电 2.5V，可从内部 LDO 稳压器或外部 LDO 芯片获取电源
#define KSDIY_MIPI_DSI_PHY_PWR_LDO_CHAN 3 // LDO_VO3 连接至 VDD_MIPI_DPHY
#define KSDIY_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define KSDIY_LCD_BK_LIGHT_ON_LEVEL 1
#define KSDIY_LCD_BK_LIGHT_OFF_LEVEL !KSDIY_LCD_BK_LIGHT_ON_LEVEL
#define KSDIY_PIN_NUM_BK_LIGHT -1
#define KSDIY_PIN_NUM_LCD_RST -1
#define KSDIY_PIN_NUM_TOUCH_IIC_SCL 8
#define KSDIY_PIN_NUM_TOUCH_IIC_SDA 7

#define CONFIG_EXAMPLE_LCD_TOUCH_ENABLED 0
static SemaphoreHandle_t lvgl_mux = NULL;
i2c_master_bus_handle_t touch_i2c_bus_;

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
#include "esp_lcd_touch_gt911.h"
#include "driver/i2c_master.h"
bool gt911_found = false;
esp_lcd_touch_handle_t tp = NULL;
esp_err_t find_gt911(void)
{
    esp_err_t ret = ESP_OK;
    for (uint8_t i = 0x01; i < 0x7F; i++)
    {
        ret = i2c_master_probe(touch_i2c_bus_, i, -1);
        if (ret == ESP_OK)
        {
            printf("The slave has been found, the address is %x\n", i);
            if (i == 0X5d)
            {
                ESP_LOGI("I2C", "找到设备，地址: 0x%02X,返回 OK", i);
                gt911_found = true;
                return ESP_OK; // 找到 0x15 地址，返回 OK
            }
        }
    }

    ESP_LOGE("I2C", "未找到地址 0x5d,返回失败");
    gt911_found = false;
    return ESP_FAIL; // 没有找到设备
}
void i2c_init(void)
{
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = KSDIY_PIN_NUM_TOUCH_IIC_SDA,
        .scl_io_num = KSDIY_PIN_NUM_TOUCH_IIC_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &touch_i2c_bus_));
    find_gt911();
}
#endif
bool ksdiy_lvgl_lock(int timeout_ms)
{
    // 如果 lvgl_port_lock 函数不可用，可以使用自定义的互斥锁
    // 这里假设 esp_lvgl_port 组件提供了这个函数
    return lvgl_port_lock(timeout_ms);
}

void ksdiy_lvgl_unlock(void)
{
    // 如果 lvgl_port_unlock 函数不可用，可以使用自定义的互斥锁
    lvgl_port_unlock();
}

static void ksdiy_bsp_enable_dsi_phy_power(void)
{
    // 打开 MIPI DSI PHY 的电源，使其从“无电”状态进入“关机”状态
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
#ifdef KSDIY_MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = KSDIY_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = KSDIY_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif
}

static void ksdiy_bsp_init_lcd_backlight(void)
{
#if KSDIY_PIN_NUM_BK_LIGHT >= 0
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << KSDIY_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif
}

static void ksdiy_bsp_set_lcd_backlight(uint32_t level)
{
#if KSDIY_PIN_NUM_BK_LIGHT >= 0
    gpio_set_level(KSDIY_PIN_NUM_BK_LIGHT, level);
#endif
}

void ksdiy_lvgl_port_init(void)
{
    ksdiy_bsp_enable_dsi_phy_power();
    ksdiy_bsp_init_lcd_backlight();
    ksdiy_bsp_set_lcd_backlight(KSDIY_LCD_BK_LIGHT_OFF_LEVEL);

    // 首先创建 MIPI DSI 总线，它还将初始化 DSI PHY
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    esp_lcd_panel_io_handle_t mipi_dbi_io;
    // 我们使用DBI接口发送LCD命令和参数
    esp_lcd_dbi_io_config_t dbi_config = PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));
    // 创建ST7703控制面板
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_dpi_panel_config_t dpi_config = PANEL_DPI_CONFIG(MIPI_DPI_PX_FORMAT);
    #ifdef LCD_TYPE_ST7703
    st7703_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = KSDIY_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_COLOR_DEPTH,
        .vendor_config = &vendor_config,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7703(mipi_dbi_io, &panel_config, &panel_handle));

    #elif defined LCD_TYPE_GC9503
    gc9503_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = KSDIY_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_COLOR_DEPTH,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9503(mipi_dbi_io, &panel_config, &panel_handle));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    i2c_init();
    if (gt911_found == true)
    {
        /************* 初始化触摸屏 **************/
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
        tp_io_config.scl_speed_hz = 400000;

        esp_lcd_touch_config_t tp_cfg = {
            .x_max = KSDIY_MIPI_DSI_LCD_H_RES,
            .y_max = KSDIY_MIPI_DSI_LCD_V_RES,
            .rst_gpio_num = -1,
            .int_gpio_num = -1,
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };

#if ((ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 4) || ESP_IDF_VERSION_MAJOR > 5)
        esp_lcd_new_panel_io_i2c_v2((i2c_master_bus_handle_t)touch_i2c_bus_, &tp_io_config, &tp_io_handle);
#else
        esp_lcd_new_panel_io_i2c_v2((esp_lcd_i2c_bus_handle_t)touch_i2c_bus_, &tp_io_config, &tp_io_handle);
#endif
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));
    }
#endif
    // 打开背光
    ksdiy_bsp_set_lcd_backlight(KSDIY_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // 初始化LVGL端口
    // 如果 lvgl_port_init 函数不可用，可以使用自定义的初始化函数
    ESP_LOGI(TAG, "Initialize LVGL port");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
    lvgl_port_init(&port_cfg);

    // 配置显示设备
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = mipi_dbi_io,
        .panel_handle = panel_handle,
        .buffer_size = KSDIY_MIPI_DSI_LCD_H_RES * KSDIY_MIPI_DSI_LCD_V_RES, // 缓冲区大小
        .double_buffer = true,                                              // 使用双缓冲
        .hres = KSDIY_MIPI_DSI_LCD_H_RES,
        .vres = KSDIY_MIPI_DSI_LCD_V_RES,
        .monochrome = false, // 是否为单色显示器
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .color_format = LVGL_PX_FORMAT,
        .flags = {
            // .buff_dma = true, // 使用DMA缓冲区
            .buff_spiram = true, // 使用DMA缓冲区
            // .full_refresh = true, // 启用全屏幕刷新
            // .swap_bytes = 1,
        }};
    // MIPI DSI特定配置
    const lvgl_port_display_dsi_cfg_t dsi_cfg = {
        .flags = {
            .avoid_tearing = false, // 启用防撕裂
        }};
    // 添加MIPI DSI显示设备
    lv_disp_t *disp = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);
    if (disp == NULL)
    {
        ESP_LOGE(TAG, "LVGL显示设备添加失败");
        return;
    }
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED

    if (tp != NULL)
    {
        // 配置触摸设备
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = disp,
            .handle = tp,
            .scale = {
                .x = 1.0,
                .y = 1.0,
            },
        };

        // 添加触摸设备到LVGL
        lv_indev_t *touch_indev = lvgl_port_add_touch(&touch_cfg);
        if (touch_indev == NULL)
        {
            ESP_LOGE(TAG, "LVGL触摸设备添加失败");
            return;
        }
    }
#endif
    ESP_LOGI(TAG, "LVGL端口初始化完成");
}
