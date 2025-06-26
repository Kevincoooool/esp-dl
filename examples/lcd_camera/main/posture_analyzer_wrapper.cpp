#include "posture_analyzer.hpp"

extern "C" {

typedef struct {
    PostureState state;
    float confidence;
    float head_tilt_angle;
    float spine_curve_angle;
    float shoulder_balance;
    bool head_low_position;
} CPostureResult;

PostureAnalyzer* posture_analyzer_create(void) {
    return new PostureAnalyzer();
}

void posture_analyzer_destroy(PostureAnalyzer* analyzer) {
    if (analyzer) {
        delete analyzer;
    }
}

int posture_analyzer_init(PostureAnalyzer* analyzer) {
    if (!analyzer) return -1;
    return analyzer->init() == ESP_OK ? 0 : -1;
}

CPostureResult posture_analyzer_analyze_frame(PostureAnalyzer* analyzer, uint8_t* buffer, size_t buffer_size, uint32_t width, uint32_t height, uint32_t format) {
    CPostureResult result = {};
    
    if (!analyzer) {
        result.state = (PostureState)4; // UNKNOWN
        return result;
    }
    
    PostureResult cpp_result = analyzer->analyze_frame(buffer, buffer_size, width, height, format);
    
    result.state = cpp_result.state;
    result.confidence = cpp_result.confidence;
    result.head_tilt_angle = cpp_result.head_tilt_angle;
    result.spine_curve_angle = cpp_result.spine_curve_angle;
    result.shoulder_balance = cpp_result.shoulder_balance;
    result.head_low_position = cpp_result.head_low_position;
    
    return result;
}

void posture_analyzer_set_thresholds(PostureAnalyzer* analyzer, float head_tilt, float lying_head, float hunch_angle, float min_conf) {
    if (analyzer) {
        analyzer->set_thresholds(head_tilt, lying_head, hunch_angle, min_conf);
    }
}

const char* posture_get_state_description(PostureState state) {
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

const char* posture_get_state_icon(PostureState state) {
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

} // extern "C" 