#pragma once

#include "coco_pose.hpp"
#include "esp_camera.h"
#include "esp_log.h"
#include "dl_image_color.hpp"
#include "dl_detect_define.hpp"
#include <vector>
#include <cmath>
#include <list>

// COCO关键点索引定义
enum class KeypointIndex {
    NOSE = 0,
    LEFT_EYE = 1,
    RIGHT_EYE = 2,
    LEFT_EAR = 3,
    RIGHT_EAR = 4,
    LEFT_SHOULDER = 5,
    RIGHT_SHOULDER = 6,
    LEFT_ELBOW = 7,
    RIGHT_ELBOW = 8,
    LEFT_WRIST = 9,
    RIGHT_WRIST = 10,
    LEFT_HIP = 11,
    RIGHT_HIP = 12,
    LEFT_KNEE = 13,
    RIGHT_KNEE = 14,
    LEFT_ANKLE = 15,
    RIGHT_ANKLE = 16
};

// 坐姿状态定义（英文）
enum class PostureState {
    NORMAL_SITTING,    // Normal Sitting
    LYING_ON_TABLE,    // Lying on Table
    HEAD_TILTED,       // Head Tilted
    HUNCHED_BACK,      // Hunched Back
    LEANING_FORWARD,   // Leaning Forward
    UNKNOWN            // Detecting...
};

// 关键点结构
struct Keypoint {
    float x;
    float y;
    float confidence;
    
    bool is_valid() const {
        return confidence > 0.3f && x >= 0 && y >= 0;
    }
};

// 坐姿检测结果
struct PostureResult {
    PostureState state;
    float confidence;
    std::vector<Keypoint> keypoints;
    
    // 分析指标
    float head_tilt_angle;
    float spine_curve_angle;
    float shoulder_balance;
    bool head_low_position;
    
    PostureResult() : state(PostureState::UNKNOWN), confidence(0.0f),
                     head_tilt_angle(0.0f), spine_curve_angle(0.0f),
                     shoulder_balance(0.0f), head_low_position(false) {}
};

class PostureAnalyzer {
private:
    COCOPose* pose_detector;
    
    // 检测阈值
    float head_tilt_threshold;
    float lying_head_threshold;
    float hunch_angle_threshold;
    float min_confidence;
    
    // 内部状态
    uint32_t frame_counter;
    PostureState current_state;
    bool ai_initialized;
    
    // 私有方法
    std::vector<Keypoint> extract_keypoints(const std::list<dl::detect::result_t>& results);
    PostureState analyze_posture(const std::vector<Keypoint>& keypoints, PostureResult& result);
    
    // 几何计算方法
    float calculate_head_tilt(const std::vector<Keypoint>& keypoints);
    float calculate_spine_curvature(const std::vector<Keypoint>& keypoints);
    float calculate_shoulder_balance(const std::vector<Keypoint>& keypoints);
    bool is_head_too_low(const std::vector<Keypoint>& keypoints);
    float calculate_angle(const Keypoint& p1, const Keypoint& p2, const Keypoint& p3);
    float calculate_distance(const Keypoint& p1, const Keypoint& p2);
    
    // 图像处理辅助方法
    dl::image::img_t convert_camera_frame(camera_fb_t* fb_data);
    
public:
    PostureAnalyzer();
    ~PostureAnalyzer();
    
    esp_err_t init();
    void set_thresholds(float head_tilt, float lying_head, float hunch_angle, float min_conf);
    PostureResult analyze_frame(camera_fb_t* fb_data);
    
    // 状态查询方法
    const char* get_state_description(PostureState state) const;
    const char* get_state_icon(PostureState state) const;
    uint32_t get_frame_count() const { return frame_counter; }
}; 