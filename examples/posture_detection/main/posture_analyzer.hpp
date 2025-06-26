#pragma once
#include <vector>
#include <cmath>

// 关键点索引（COCO格式）
enum KeypointIndex {
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

// 坐姿状态枚举
enum PostureState {
    NORMAL_SITTING = 0,    // 正常坐姿
    LYING_DOWN = 1,        // 趴桌
    HEAD_TILTED = 2,       // 歪头
    HUNCHED_BACK = 3,      // 驼背
    UNKNOWN = 4            // 未知状态
};

// 关键点结构
struct Keypoint {
    float x, y;
    float confidence;
};

class PostureAnalyzer {
private:
    float head_tilt_threshold = 15.0;      // 头部倾斜阈值（度）
    float lying_height_ratio = 0.3;       // 趴桌高度比例阈值
    float shoulder_slope_threshold = 10.0;  // 肩膀倾斜阈值（度）
    
public:
    PostureAnalyzer() {}
    
    // 分析坐姿状态
    PostureState analyzePosture(const std::vector<Keypoint>& keypoints);
    
    // 计算头部倾斜角度
    float calculateHeadTilt(const std::vector<Keypoint>& keypoints);
    
    // 检测是否趴桌
    bool isLyingDown(const std::vector<Keypoint>& keypoints);
    
    // 检测驼背
    bool isHunchedBack(const std::vector<Keypoint>& keypoints);
    
    // 获取状态描述
    const char* getPostureDescription(PostureState state);
    
    // 设置检测阈值
    void setHeadTiltThreshold(float threshold) { head_tilt_threshold = threshold; }
    void setLyingHeightRatio(float ratio) { lying_height_ratio = ratio; }
    void setShoulderSlopeThreshold(float threshold) { shoulder_slope_threshold = threshold; }
}; 