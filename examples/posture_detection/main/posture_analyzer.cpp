#include "posture_analyzer.hpp"
#include <cmath>
#include <algorithm>

PostureState PostureAnalyzer::analyzePosture(const std::vector<Keypoint>& keypoints) {
    if (keypoints.size() < 17) {
        return UNKNOWN;
    }
    
    // 检查关键点置信度
    if (keypoints[NOSE].confidence < 0.5 || 
        keypoints[LEFT_SHOULDER].confidence < 0.5 || 
        keypoints[RIGHT_SHOULDER].confidence < 0.5) {
        return UNKNOWN;
    }
    
    // 1. 检测趴桌（优先级最高）
    if (isLyingDown(keypoints)) {
        return LYING_DOWN;
    }
    
    // 2. 检测头部倾斜
    float head_tilt = calculateHeadTilt(keypoints);
    if (std::abs(head_tilt) > head_tilt_threshold) {
        return HEAD_TILTED;
    }
    
    // 3. 检测驼背
    if (isHunchedBack(keypoints)) {
        return HUNCHED_BACK;
    }
    
    // 4. 正常坐姿
    return NORMAL_SITTING;
}

float PostureAnalyzer::calculateHeadTilt(const std::vector<Keypoint>& keypoints) {
    // 使用双眼或双耳计算头部倾斜角度
    float left_x, left_y, right_x, right_y;
    
    if (keypoints[LEFT_EYE].confidence > 0.5 && keypoints[RIGHT_EYE].confidence > 0.5) {
        left_x = keypoints[LEFT_EYE].x;
        left_y = keypoints[LEFT_EYE].y;
        right_x = keypoints[RIGHT_EYE].x;
        right_y = keypoints[RIGHT_EYE].y;
    } else if (keypoints[LEFT_EAR].confidence > 0.5 && keypoints[RIGHT_EAR].confidence > 0.5) {
        left_x = keypoints[LEFT_EAR].x;
        left_y = keypoints[LEFT_EAR].y;
        right_x = keypoints[RIGHT_EAR].x;
        right_y = keypoints[RIGHT_EAR].y;
    } else {
        return 0.0; // 无法计算
    }
    
    // 计算倾斜角度
    float dy = right_y - left_y;
    float dx = right_x - left_x;
    float angle = atan2(dy, dx) * 180.0 / M_PI;
    
    return angle;
}

bool PostureAnalyzer::isLyingDown(const std::vector<Keypoint>& keypoints) {
    // 基于鼻子和肩膀的相对位置判断是否趴桌
    float nose_y = keypoints[NOSE].y;
    float left_shoulder_y = keypoints[LEFT_SHOULDER].y;
    float right_shoulder_y = keypoints[RIGHT_SHOULDER].y;
    float avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2.0;
    
    // 如果鼻子位置接近或低于肩膀，认为是趴桌
    float height_ratio = (avg_shoulder_y - nose_y) / avg_shoulder_y;
    
    return height_ratio < lying_height_ratio;
}

bool PostureAnalyzer::isHunchedBack(const std::vector<Keypoint>& keypoints) {
    // 检查肩膀是否前倾（简化版检测）
    // 可以通过肩膀与其他关键点的相对位置来判断
    
    // 这里实现一个简化的检测逻辑
    // 实际应用中可能需要更复杂的算法
    
    float left_shoulder_x = keypoints[LEFT_SHOULDER].x;
    float right_shoulder_x = keypoints[RIGHT_SHOULDER].x;
    float nose_x = keypoints[NOSE].x;
    
    // 计算肩膀中点
    float shoulder_center_x = (left_shoulder_x + right_shoulder_x) / 2.0;
    
    // 如果肩膀中点明显偏离鼻子位置，可能是驼背
    float offset_ratio = std::abs(shoulder_center_x - nose_x) / std::abs(right_shoulder_x - left_shoulder_x);
    
    return offset_ratio > 0.3; // 阈值可调整
}

const char* PostureAnalyzer::getPostureDescription(PostureState state) {
    switch (state) {
        case NORMAL_SITTING: return "正常坐姿";
        case LYING_DOWN: return "趴桌";
        case HEAD_TILTED: return "歪头";
        case HUNCHED_BACK: return "驼背";
        default: return "未知状态";
    }
} 