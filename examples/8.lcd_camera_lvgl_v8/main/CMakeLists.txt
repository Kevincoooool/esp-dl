set(srcs 
    "app_main.cpp" 
    "app_camera.c" 
    "ksdiy_lvgl_port.c" 
    "page_cam.cpp" 
    "posture_analyzer.cpp"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES esp32-camera esp_lcd lvgl driver esp_timer freertos esp_common nvs_flash coco_pose
)

# 忽略ESP-DL库的编译警告
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Wno-unused-but-set-variable
    -Wno-unused-variable
    -Wno-array-bounds
)
