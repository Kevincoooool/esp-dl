#include "posture_analyzer.hpp"
#include "dl_image_color.hpp"
#include "dl_image_process.hpp"
#include "linux/videodev2.h"

static const char* TAG = "PostureAnalyzer_P4";

PostureAnalyzer::PostureAnalyzer() : 
    pose_detector(nullptr),
    head_tilt_threshold(20.0f),
    lying_head_threshold(0.7f),
    hunch_angle_threshold(25.0f),
    min_confidence(0.4f),
    frame_counter(0),
    current_state(PostureState::UNKNOWN),
    ai_initialized(false)
{
    // 预分配内存池以减少动态分配开销
    conversion_buffer = nullptr;
    buffer_size = 0;
    ESP_LOGI(TAG, "ESP32-P4 Real PostureAnalyzer created");
}

PostureAnalyzer::~PostureAnalyzer() {
    if (pose_detector) {
        delete pose_detector;
        pose_detector = nullptr;
    }
    
    // 释放内存池
    if (conversion_buffer) {
        heap_caps_free(conversion_buffer);
        conversion_buffer = nullptr;
    }
    
    ESP_LOGI(TAG, "PostureAnalyzer destroyed");
}

esp_err_t PostureAnalyzer::init() {
    ESP_LOGI(TAG, "Initializing YOLO11n-pose PostureAnalyzer for ESP32-P4...");
    
    // 创建YOLO11n-pose模型实例
    pose_detector = new COCOPose();
    
    if (!pose_detector) {
        ESP_LOGE(TAG, "Failed to create COCOPose instance");
        return ESP_FAIL;
    }
    
    ai_initialized = true;
    frame_counter = 0;
    current_state = PostureState::UNKNOWN;
    
    ESP_LOGI(TAG, "✅ YOLO11n-pose model initialized successfully for ESP32-P4");
    ESP_LOGI(TAG, "- Head tilt threshold: %.1f°", head_tilt_threshold);
    ESP_LOGI(TAG, "- Lying detection threshold: %.2f", lying_head_threshold);
    ESP_LOGI(TAG, "- Hunch angle threshold: %.1f°", hunch_angle_threshold);
    ESP_LOGI(TAG, "- Minimum confidence: %.2f", min_confidence);
    
    return ESP_OK;
}

void PostureAnalyzer::set_thresholds(float head_tilt, float lying_head, float hunch_angle, float min_conf) {
    head_tilt_threshold = head_tilt;
    lying_head_threshold = lying_head;
    hunch_angle_threshold = hunch_angle;
    min_confidence = min_conf;
    
    ESP_LOGI(TAG, "Thresholds updated - Head tilt: %.1f°, Lying: %.2f, Hunch: %.1f°, Min conf: %.2f",
             head_tilt, lying_head, hunch_angle, min_conf);
}

PostureResult PostureAnalyzer::analyze_frame(uint8_t* buffer, size_t buffer_size, uint32_t width, uint32_t height, uint32_t format) {
    PostureResult result;
    
    if (!ai_initialized || !pose_detector || !buffer) {
        result.state = PostureState::UNKNOWN;
        result.confidence = 0.0f;
        return result;
    }
    
    frame_counter++;
    
    // 跳帧策略：每3帧才检测一次（外部已经是每20帧了）
    if (frame_counter % 3 != 0) {
        // 返回上次的结果，不进行AI推理
        result.state = current_state;
        result.confidence = 0.8f; // 使用缓存结果的置信度
        return result;
    }
    
    // 转换摄像头帧为dl::image格式
    dl::image::img_t img = convert_frame_to_rgb888(buffer, buffer_size, width, height, format);
    if (!img.data) {
        ESP_LOGE(TAG, "Failed to convert camera frame");
        result.state = PostureState::UNKNOWN;
        return result;
    }
    
    // 运行YOLO11n-pose检测
    ESP_LOGI(TAG, "🚀 Starting YOLO11n-pose detection on %dx%d image (frame %lu)...", img.width, img.height, frame_counter);
    
    uint32_t model_start = esp_timer_get_time() / 1000;
    std::list<dl::detect::result_t>& pose_results = pose_detector->run(img);
    uint32_t model_time = (esp_timer_get_time() / 1000) - model_start;
    
    ESP_LOGI(TAG, "✅ YOLO11n-pose model inference completed in %lums, found %d results", 
             model_time, pose_results.size());
    
    // 注意：img.data现在指向内存池，不需要释放
    
    if (pose_results.empty()) {
        result.state = PostureState::UNKNOWN;
        result.confidence = 0.0f;
        ESP_LOGI(TAG, "No person detected in frame %lu", frame_counter);
        return result;
    }
    
    // 提取关键点（选择置信度最高的检测结果）
    std::vector<Keypoint> keypoints = extract_keypoints(pose_results);
    result.keypoints = keypoints;
    
    // 分析坐姿
    result.state = analyze_posture(keypoints, result);
    
    // 计算整体置信度
    float total_confidence = 0.0f;
    int valid_points = 0;
    for (const auto& kp : keypoints) {
        if (kp.is_valid()) {
            total_confidence += kp.confidence;
            valid_points++;
        }
    }
    result.confidence = valid_points > 0 ? (total_confidence / valid_points) : 0.0f;
    
    current_state = result.state;
    
    ESP_LOGI(TAG, "Frame %lu: State=%s, Confidence=%.2f, Head Tilt=%.1f°, Spine=%.1f°",
             frame_counter,
             get_state_description(result.state),
             result.confidence,
             result.head_tilt_angle,
             result.spine_curve_angle);
    
    return result;
}

dl::image::img_t PostureAnalyzer::convert_frame_to_rgb888(uint8_t* buffer, size_t buffer_size, uint32_t width, uint32_t height, uint32_t format) {
    dl::image::img_t img = {};
    img.data = nullptr;
    
    if (!buffer || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid frame data");
        return img;
    }
    
    ESP_LOGI(TAG, "📥 Input frame: %ld×%ld, format=0x%lx, size=%zu bytes", 
             width, height, format, buffer_size);
    
    // 处理RGB565格式 - 使用ESP-DL优化版本
    if (format == V4L2_PIX_FMT_RGB565) {
        // 直接输出640×640以匹配YOLO11n-pose模型期望输入，避免模型内部缩放
        uint32_t new_width = 320;
        uint32_t new_height = 320;
        
        ESP_LOGI(TAG, "🔄 Converting frame: %ld x %ld -> %ld x %ld (model native)", 
                 width, height, new_width, new_height);
        
        // 计算居中裁剪区域 - 保持图像比例，从1280×720裁剪为正方形
        int crop_size = std::min(width, height);  // 720
        int crop_x = (width - crop_size) / 2;     // (1280-720)/2 = 280
        int crop_y = (height - crop_size) / 2;    // (720-720)/2 = 0
        std::vector<int> crop_area = {crop_x, crop_y, crop_x + crop_size, crop_y + crop_size};
        
        ESP_LOGI(TAG, "✂️ Center crop area: [%d, %d, %d, %d] (crop_size=%d)", 
                 crop_x, crop_y, crop_x + crop_size, crop_y + crop_size, crop_size);
        
        // 创建源图像结构
        dl::image::img_t src_img = {
            .data = buffer,
            .width = (uint16_t)width,
            .height = (uint16_t)height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565
        };
        
        // 使用内存池避免频繁分配
        size_t rgb888_size = new_width * new_height * 3;
        uint8_t* rgb888_data;
        if (!conversion_buffer || buffer_size < rgb888_size) {
            if (conversion_buffer) {
                heap_caps_free(conversion_buffer);
            }
            conversion_buffer = (uint8_t*)heap_caps_malloc(rgb888_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            buffer_size = rgb888_size;
            ESP_LOGI(TAG, "💾 Allocated new conversion buffer: %zu bytes", rgb888_size);
        }
        rgb888_data = conversion_buffer;
        
        if (!rgb888_data) {
            ESP_LOGE(TAG, "Failed to allocate RGB888 buffer");
            return img;
        }
        
        // 创建目标图像结构
        dl::image::img_t dst_img = {
            .data = rgb888_data,
            .width = (uint16_t)new_width,
            .height = (uint16_t)new_height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888
        };
        
        ESP_LOGI(TAG, "⏱️ Starting ESP-DL resize operation...");
        uint32_t resize_start = esp_timer_get_time() / 1000;
        
        // 使用ESP-DL的优化resize函数进行转换和缩放
        dl::image::resize(src_img, dst_img, dl::image::DL_IMAGE_INTERPOLATE_NEAREST, 0, nullptr, crop_area);
        
        uint32_t resize_time = (esp_timer_get_time() / 1000) - resize_start;
        ESP_LOGI(TAG, "✅ ESP-DL resize completed in %lums: %dx%d -> %dx%d", 
                 resize_time, src_img.width, src_img.height, dst_img.width, dst_img.height);
        
        img.data = rgb888_data;
        img.width = new_width;
        img.height = new_height;
        
    } else {
        ESP_LOGE(TAG, "Unsupported pixel format: 0x%lx", format);
    }
    
    return img;
}

std::vector<Keypoint> PostureAnalyzer::extract_keypoints(const std::list<dl::detect::result_t>& results) {
    std::vector<Keypoint> keypoints(17);
    
    if (results.empty()) {
        return keypoints;
    }
    
    // 选择置信度最高的人物检测结果
    auto best_iter = results.begin();
    for (auto iter = results.begin(); iter != results.end(); ++iter) {
        if (iter->score > best_iter->score) {
            best_iter = iter;
        }
    }
    
    const dl::detect::result_t& best_result = *best_iter;
    
    // 提取17个关键点
    for (int i = 0; i < 17 && i * 2 + 1 < best_result.keypoint.size(); i++) {
        keypoints[i].x = best_result.keypoint[2 * i];
        keypoints[i].y = best_result.keypoint[2 * i + 1];
        keypoints[i].confidence = best_result.score; // 使用整体检测置信度
    }
    
    return keypoints;
}

PostureState PostureAnalyzer::analyze_posture(const std::vector<Keypoint>& keypoints, PostureResult& result) {
    // 计算分析指标
    result.head_tilt_angle = calculate_head_tilt(keypoints);
    result.spine_curve_angle = calculate_spine_curvature(keypoints);
    result.shoulder_balance = calculate_shoulder_balance(keypoints);
    result.head_low_position = is_head_too_low(keypoints);
    
    // 分析逻辑
    if (result.head_low_position && result.spine_curve_angle > 30.0f) {
        return PostureState::LYING_ON_TABLE;
    } else if (abs(result.head_tilt_angle) > head_tilt_threshold) {
        return PostureState::HEAD_TILTED;
    } else if (result.spine_curve_angle > hunch_angle_threshold) {
        return PostureState::HUNCHED_BACK;
    } else if (result.spine_curve_angle > 15.0f && !result.head_low_position) {
        return PostureState::LEANING_FORWARD;
    } else {
        return PostureState::NORMAL_SITTING;
    }
}

float PostureAnalyzer::calculate_head_tilt(const std::vector<Keypoint>& keypoints) {
    if (keypoints.size() < 17) return 0.0f;
    
    const Keypoint& left_ear = keypoints[(int)KeypointIndex::LEFT_EAR];
    const Keypoint& right_ear = keypoints[(int)KeypointIndex::RIGHT_EAR];
    
    if (!left_ear.is_valid() || !right_ear.is_valid()) {
        return 0.0f;
    }
    
    float dx = right_ear.x - left_ear.x;
    float dy = right_ear.y - left_ear.y;
    float angle = atan2(dy, dx) * 180.0f / M_PI;
    
    return angle;
}

float PostureAnalyzer::calculate_spine_curvature(const std::vector<Keypoint>& keypoints) {
    if (keypoints.size() < 17) return 0.0f;
    
    const Keypoint& nose = keypoints[(int)KeypointIndex::NOSE];
    const Keypoint& left_shoulder = keypoints[(int)KeypointIndex::LEFT_SHOULDER];
    const Keypoint& right_shoulder = keypoints[(int)KeypointIndex::RIGHT_SHOULDER];
    const Keypoint& left_hip = keypoints[(int)KeypointIndex::LEFT_HIP];
    const Keypoint& right_hip = keypoints[(int)KeypointIndex::RIGHT_HIP];
    
    if (!nose.is_valid() || !left_shoulder.is_valid() || !right_shoulder.is_valid() ||
        !left_hip.is_valid() || !right_hip.is_valid()) {
        return 0.0f;
    }
    
    // 计算肩膀和臀部中点
    Keypoint shoulder_mid = {
        (left_shoulder.x + right_shoulder.x) / 2,
        (left_shoulder.y + right_shoulder.y) / 2,
        (left_shoulder.confidence + right_shoulder.confidence) / 2
    };
    
    Keypoint hip_mid = {
        (left_hip.x + right_hip.x) / 2,
        (left_hip.y + right_hip.y) / 2,
        (left_hip.confidence + right_hip.confidence) / 2
    };
    
    // 计算脊柱角度
    return calculate_angle(nose, shoulder_mid, hip_mid);
}

float PostureAnalyzer::calculate_shoulder_balance(const std::vector<Keypoint>& keypoints) {
    if (keypoints.size() < 17) return 0.0f;
    
    const Keypoint& left_shoulder = keypoints[(int)KeypointIndex::LEFT_SHOULDER];
    const Keypoint& right_shoulder = keypoints[(int)KeypointIndex::RIGHT_SHOULDER];
    
    if (!left_shoulder.is_valid() || !right_shoulder.is_valid()) {
        return 0.0f;
    }
    
    return abs(left_shoulder.y - right_shoulder.y);
}

bool PostureAnalyzer::is_head_too_low(const std::vector<Keypoint>& keypoints) {
    if (keypoints.size() < 17) return false;
    
    const Keypoint& nose = keypoints[(int)KeypointIndex::NOSE];
    const Keypoint& left_shoulder = keypoints[(int)KeypointIndex::LEFT_SHOULDER];
    const Keypoint& right_shoulder = keypoints[(int)KeypointIndex::RIGHT_SHOULDER];
    
    if (!nose.is_valid() || !left_shoulder.is_valid() || !right_shoulder.is_valid()) {
        return false;
    }
    
    float shoulder_y = (left_shoulder.y + right_shoulder.y) / 2;
    float head_shoulder_ratio = (nose.y - shoulder_y) / abs(left_shoulder.y - right_shoulder.y + 1);
    
    return head_shoulder_ratio < lying_head_threshold;
}

float PostureAnalyzer::calculate_angle(const Keypoint& p1, const Keypoint& p2, const Keypoint& p3) {
    float dx1 = p1.x - p2.x;
    float dy1 = p1.y - p2.y;
    float dx2 = p3.x - p2.x;
    float dy2 = p3.y - p2.y;
    
    float dot = dx1 * dx2 + dy1 * dy2;
    float det = dx1 * dy2 - dy1 * dx2;
    
    float angle = atan2(det, dot) * 180.0f / M_PI;
    return abs(angle);
}

float PostureAnalyzer::calculate_distance(const Keypoint& p1, const Keypoint& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

const char* PostureAnalyzer::get_state_description(PostureState state) const {
    switch (state) {
        case PostureState::NORMAL_SITTING:   return "Normal Sitting";
        case PostureState::LYING_ON_TABLE:   return "Lying on Table";
        case PostureState::HEAD_TILTED:      return "Head Tilted";
        case PostureState::HUNCHED_BACK:     return "Hunched Back";
        case PostureState::LEANING_FORWARD:  return "Leaning Forward";
        case PostureState::UNKNOWN:          return "Detecting...";
        default:                             return "Unknown State";
    }
}

const char* PostureAnalyzer::get_state_icon(PostureState state) const {
    switch (state) {
        case PostureState::NORMAL_SITTING:   return "✅";
        case PostureState::LYING_ON_TABLE:   return "😴";
        case PostureState::HEAD_TILTED:      return "🤔";
        case PostureState::HUNCHED_BACK:     return "🐛";
        case PostureState::LEANING_FORWARD:  return "📱";
        case PostureState::UNKNOWN:          return "🔍";
        default:                             return "❓";
    }
} 