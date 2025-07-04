idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()
set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)

set(src_dirs        .)

set(include_dirs    .)

set(requires        esp-dl)

set(packed_model ${BUILD_DIR}/espdl_models/coco_pose.espdl)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

if (NOT CONFIG_COCO_POSE_MODEL_IN_SDCARD)
    if (IDF_TARGET STREQUAL "esp32s3")
        set(models_dir ${COMPONENT_DIR}/models/s3)
    elseif (IDF_TARGET STREQUAL "esp32p4")
        set(models_dir ${COMPONENT_DIR}/models/p4)
    endif()

    file(MAKE_DIRECTORY ${BUILD_DIR}/espdl_models)
    set(models)
    if(CONFIG_COCO_POSE_YOLO11N_POSE_S8_V1)
        list(APPEND models ${models_dir}/coco_pose_yolo11n_pose_s8_v1.espdl)
    endif()
    if(CONFIG_COCO_POSE_YOLO11N_POSE_S8_V2)
        list(APPEND models ${models_dir}/coco_pose_yolo11n_pose_s8_v2.espdl)
    endif()
    if(CONFIG_COCO_POSE_YOLO11N_POSE_320_P4_V3)
        list(APPEND models ${models_dir}/coco_pose_yolo11n_pose_320_p4_v3.espdl)
    endif()
    if(CONFIG_COCO_POSE_YOLO11N_POSE_256_P4)
        list(APPEND models ${models_dir}/coco_pose_yolo11n_pose_256_p4.espdl)
    endif()
    if(CONFIG_COCO_POSE_YOLO11N_POSE_224_P4)
        list(APPEND models ${models_dir}/coco_pose_yolo11n_pose_224_p4.espdl)
    endif()

    set(pack_model_exe ${espdl_dir}/fbs_loader/pack_espdl_models.py)
    add_custom_command(
        OUTPUT ${packed_model}
        COMMENT "Move and Pack models..."
        COMMAND python ${pack_model_exe} --model_path ${models} --out_file ${packed_model}
        DEPENDS ${models}
        VERBATIM)

    if(CONFIG_COCO_POSE_MODEL_IN_FLASH_RODATA)
        target_add_aligned_binary_data(${COMPONENT_LIB} ${packed_model} BINARY)
    endif()

    if(CONFIG_COCO_POSE_MODEL_IN_FLASH_PARTITION)
        add_custom_target(COCO_POSE_MODEL ALL DEPENDS ${packed_model})
        add_dependencies(flash COCO_POSE_MODEL)
        esptool_py_flash_to_partition(flash "coco_pose" ${packed_model})
    endif()
endif()
