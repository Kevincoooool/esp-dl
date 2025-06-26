idf_component_register(
    SRCS
        "app_main.c"
        "app_video.c"
        "ksdiy_lvgl_port.c"
        "posture_analyzer.cpp"
        "posture_analyzer_wrapper.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp-dl
        coco_pose
    )

