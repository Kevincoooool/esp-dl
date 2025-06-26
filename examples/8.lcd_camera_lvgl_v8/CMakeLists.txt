# ESP32-S3增强版坐姿检测摄像头项目
cmake_minimum_required(VERSION 3.16)

# 设置全局编译选项以忽略ESP-DL库的警告
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-variable -Wno-unused-variable -Wno-array-bounds")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable -Wno-unused-variable -Wno-array-bounds")

# 注释掉ESP-DL组件路径（演示版本不需要）
# set(EXTRA_COMPONENT_DIRS 
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../esp-dl
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../models/coco_pose
# )

# 编译选项
add_compile_options(-Wno-error=format= -Wno-format)
add_compile_options(-Wno-error=unused-function)
add_compile_options(-Wno-error=unused-variable)
add_compile_options(-Wno-error=unused-but-set-variable)
add_compile_options(-Wno-unused-variable)
add_compile_options(-Wno-unused-function)

# 设置分区表
set(PARTITION_CSV_PATH ${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(posture_detection_camera VERSION 3.0.0 DESCRIPTION "ESP32-S3增强版坐姿检测摄像头项目")
