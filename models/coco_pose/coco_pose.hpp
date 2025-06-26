#pragma once
#include "dl_detect_base.hpp"
#include "dl_pose_yolo11_postprocessor.hpp"

namespace coco_pose {
class Yolo11nPose : public dl::detect::DetectImpl {
public:
    Yolo11nPose(const char *model_name);
};
} // namespace coco_pose

class COCOPose : public dl::detect::DetectWrapper {
public:
    typedef enum {
        YOLO11N_POSE_S8_V1,
        YOLO11N_POSE_S8_V2,
        YOLO11N_POSE_320_S8_V2,
        YOLO11N_POSE_320_P4_V3,
        YOLO11N_POSE_256_P4,
        YOLO11N_POSE_224_P4,
    } model_type_t;
    COCOPose(model_type_t model_type = static_cast<model_type_t>(CONFIG_DEFAULT_COCO_POSE_MODEL));
};
