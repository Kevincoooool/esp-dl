#include "posture_analyzer.hpp"
#include "dl_image_color.hpp"

static const char* TAG = "PostureAnalyzer";

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
    ESP_LOGI(TAG, "Real PostureAnalyzer created");
}

PostureAnalyzer::~PostureAnalyzer() {
    if (pose_detector) {
        delete pose_detector;
        pose_detector = nullptr;
    }
    ESP_LOGI(TAG, "PostureAnalyzer destroyed");
}

esp_err_t PostureAnalyzer::init() {
    ESP_LOGI(TAG, "Initializing YOLO11n-pose PostureAnalyzer...");
    
    // 创建YOLO11n-pose模型实例
    pose_detector = new COCOPose();
    
    if (!pose_detector) {
        ESP_LOGE(TAG, "Failed to create COCOPose instance");
        return ESP_FAIL;
    }
    
    ai_initialized = true;
    frame_counter = 0;
    current_state = PostureState::UNKNOWN;
    
    ESP_LOGI(TAG, "✅ YOLO11n-pose model initialized successfully");
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

PostureResult PostureAnalyzer::analyze_frame(camera_fb_t* fb_data) {
    PostureResult result;
    
    if (!ai_initialized || !pose_detector || !fb_data) {
        result.state = PostureState::UNKNOWN;
        result.confidence = 0.0f;
        return result;
    }
    
    frame_counter++;
    
    // 转换摄像头帧为dl::image格式
    dl::image::img_t img = convert_camera_frame(fb_data);
    if (!img.data) {
        ESP_LOGE(TAG, "Failed to convert camera frame");
        result.state = PostureState::UNKNOWN;
        return result;
    }
    
    // 运行YOLO11n-pose检测
    ESP_LOGI(TAG, "Running YOLO11n-pose detection on %dx%d image...", img.width, img.height);
    
    std::list<dl::detect::result_t>& pose_results = pose_detector->run(img);
    
    ESP_LOGI(TAG, "YOLO11n-pose detection completed, found %d results", pose_results.size());
    
    // 释放图像内存
    heap_caps_free(img.data);
    
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

dl::image::img_t PostureAnalyzer::convert_camera_frame(camera_fb_t* fb_data) {
    dl::image::img_t img = {};
    img.data = nullptr;
    
    if (!fb_data || !fb_data->buf) {
        ESP_LOGE(TAG, "Invalid camera frame data");
        return img;
    }
    
    // 摄像头现在输出JPEG格式，直接解码
    if (fb_data->format == PIXFORMAT_JPEG) {
        dl::image::jpeg_img_t jpeg_img = {
            .data = (void*)fb_data->buf,
            .data_len = fb_data->len
        };
        
        ESP_LOGI(TAG, "Decoding JPEG image %dx%d, size: %zu bytes", 
                 fb_data->width, fb_data->height, fb_data->len);
        
        img = sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
        
        if (!img.data) {
            ESP_LOGE(TAG, "Failed to decode JPEG image");
            return img;
        }
        
        ESP_LOGI(TAG, "JPEG decoded successfully: %dx%d RGB888", img.width, img.height);
        
    } else {
        ESP_LOGE(TAG, "Unsupported pixel format: %d (expected JPEG)", fb_data->format);
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
    const Keypoint& left_eye = keypoints[static_cast<int>(KeypointIndex::LEFT_EYE)];
    const Keypoint& right_eye = keypoints[static_cast<int>(KeypointIndex::RIGHT_EYE)];
    
    if (!left_eye.is_valid() || !right_eye.is_valid()) return 0.0f;
    
    float dx = right_eye.x - left_eye.x;
    float dy = right_eye.y - left_eye.y;
    return atan2(dy, dx) * 180.0f / M_PI;
}

float PostureAnalyzer::calculate_spine_curvature(const std::vector<Keypoint>& keypoints) {
    const Keypoint& nose = keypoints[static_cast<int>(KeypointIndex::NOSE)];
    const Keypoint& left_shoulder = keypoints[static_cast<int>(KeypointIndex::LEFT_SHOULDER)];
    const Keypoint& right_shoulder = keypoints[static_cast<int>(KeypointIndex::RIGHT_SHOULDER)];
    const Keypoint& left_hip = keypoints[static_cast<int>(KeypointIndex::LEFT_HIP)];
    const Keypoint& right_hip = keypoints[static_cast<int>(KeypointIndex::RIGHT_HIP)];
    
    if (!nose.is_valid() || !left_shoulder.is_valid() || !right_shoulder.is_valid() ||
        !left_hip.is_valid() || !right_hip.is_valid()) return 0.0f;
    
    // 计算肩膀中点和髋部中点
    Keypoint shoulder_mid = {(left_shoulder.x + right_shoulder.x) / 2,
                            (left_shoulder.y + right_shoulder.y) / 2, 1.0f};
    Keypoint hip_mid = {(left_hip.x + right_hip.x) / 2,
                       (left_hip.y + right_hip.y) / 2, 1.0f};
    
    // 计算脊柱角度
    return calculate_angle(nose, shoulder_mid, hip_mid);
}

float PostureAnalyzer::calculate_shoulder_balance(const std::vector<Keypoint>& keypoints) {
    const Keypoint& left_shoulder = keypoints[static_cast<int>(KeypointIndex::LEFT_SHOULDER)];
    const Keypoint& right_shoulder = keypoints[static_cast<int>(KeypointIndex::RIGHT_SHOULDER)];
    
    if (!left_shoulder.is_valid() || !right_shoulder.is_valid()) return 0.0f;
    
    return abs(left_shoulder.y - right_shoulder.y);
}

bool PostureAnalyzer::is_head_too_low(const std::vector<Keypoint>& keypoints) {
    const Keypoint& nose = keypoints[static_cast<int>(KeypointIndex::NOSE)];
    const Keypoint& left_shoulder = keypoints[static_cast<int>(KeypointIndex::LEFT_SHOULDER)];
    const Keypoint& right_shoulder = keypoints[static_cast<int>(KeypointIndex::RIGHT_SHOULDER)];
    
    if (!nose.is_valid() || !left_shoulder.is_valid() || !right_shoulder.is_valid()) return false;
    
    float shoulder_avg_y = (left_shoulder.y + right_shoulder.y) / 2;
    return (nose.y - shoulder_avg_y) > lying_head_threshold;
}

float PostureAnalyzer::calculate_angle(const Keypoint& p1, const Keypoint& p2, const Keypoint& p3) {
    float dx1 = p1.x - p2.x;
    float dy1 = p1.y - p2.y;
    float dx2 = p3.x - p2.x;
    float dy2 = p3.y - p2.y;
    
    float dot = dx1 * dx2 + dy1 * dy2;
    float mag1 = sqrt(dx1 * dx1 + dy1 * dy1);
    float mag2 = sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (mag1 == 0 || mag2 == 0) return 0.0f;
    
    float cos_angle = dot / (mag1 * mag2);
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
    
    return acos(cos_angle) * 180.0f / M_PI;
}

float PostureAnalyzer::calculate_distance(const Keypoint& p1, const Keypoint& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

const char* PostureAnalyzer::get_state_description(PostureState state) const {
    switch (state) {
        case PostureState::NORMAL_SITTING:    return "Normal";
        case PostureState::LYING_ON_TABLE:    return "Lying";
        case PostureState::HEAD_TILTED:       return "Head tilted";
        case PostureState::HUNCHED_BACK:      return "Hunch";
        case PostureState::LEANING_FORWARD:   return "Lean";
        case PostureState::UNKNOWN:
        default:                              return "Detecting...";
    }
}

const char* PostureAnalyzer::get_state_icon(PostureState state) const {
    switch (state) {
        case PostureState::NORMAL_SITTING:    return "✅";
        case PostureState::LYING_ON_TABLE:    return "😴";
        case PostureState::HEAD_TILTED:       return "🤔";
        case PostureState::HUNCHED_BACK:      return "🐢";
        case PostureState::LEANING_FORWARD:   return "⬆️";
        case PostureState::UNKNOWN:
        default:                              return "❓";
    }
} 