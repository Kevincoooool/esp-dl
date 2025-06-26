idf_component_register(SRCS "app_main.cpp"
                              "posture_analyzer.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp-dl esp_camera) 