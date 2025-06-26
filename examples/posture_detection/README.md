# ESP-DL 坐姿检测示例

本示例演示如何使用 ESP-DL 实现学生坐姿检测系统，可以检测以下状态：

- ✅ **正常坐姿**：头部端正，肩膀平衡
- ❌ **趴桌**：头部过低，接近桌面  
- ❌ **歪头**：头部明显倾斜
- ❌ **驼背**：肩膀前倾，姿态不正

## 硬件要求

- ESP32-S3 或 ESP32-P4 开发板
- OV2640 摄像头模块
- 8MB+ PSRAM（推荐）

## 快速开始

### 1. 设置 ESP-IDF 环境

```bash
cd F:\Espressif\frameworks\esp-idf-v5.4.1
.\export.ps1
```

### 2. 编译和烧录

```bash
cd F:\GitHub\esp-dl\examples\posture_detection

# 配置项目（可选）
idf.py menuconfig

# 编译
idf.py build

# 烧录并监控
idf.py flash monitor
```

### 3. 查看输出

系统会每2秒输出一次检测结果：

```
I (1234) PostureDetection: ✅ 检测结果: 正常坐姿
I (1234) PostureDetection: 头部倾斜角度: 2.3°
```

## 文件结构

```
posture_detection/
├── main/
│   ├── app_main.cpp          # 主程序
│   ├── posture_analyzer.hpp  # 坐姿分析器头文件
│   ├── posture_analyzer.cpp  # 坐姿分析器实现
│   └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig.defaults        # 默认配置
└── README.md
```

## 检测原理

### 关键点检测
使用17个COCO关键点：
- 头部：鼻子、眼睛、耳朵
- 上半身：肩膀、手肘、手腕
- 下半身：臀部、膝盖、脚踝

### 算法逻辑

1. **趴桌检测**：比较鼻子与肩膀的相对高度
2. **歪头检测**：计算双眼或双耳连线的倾斜角度
3. **驼背检测**：分析肩膀与头部的相对位置

## 自定义配置

### 调整检测阈值

在 `app_main.cpp` 中修改：

```cpp
// 可调节的检测参数
analyzer.setHeadTiltThreshold(12.0);   // 头部倾斜阈值（度）
analyzer.setLyingHeightRatio(0.25);    // 趴桌检测阈值
```

### 修改相机配置

根据实际硬件调整 `camera_config` 结构体中的引脚配置。

## 当前版本说明

⚠️ **注意**：当前版本使用演示数据，实际部署需要：

1. **集成真实的模型推理**：
   - 加载量化后的 YOLO11n-pose 模型
   - 实现图像预处理和后处理
   - 集成 ESP-DL 模型推理 API

2. **优化性能**：
   - 调整模型输入尺寸
   - 优化检测频率
   - 实现多帧平滑

## 扩展功能

### 添加警告机制

```cpp
if (state != NORMAL_SITTING) {
    // 控制 LED 指示灯
    gpio_set_level(LED_PIN, 1);
    
    // 蜂鸣器警告
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(BUZZER_PIN, 0);
}
```

### 数据记录

```cpp
// 记录不良姿势时间
if (state != NORMAL_SITTING) {
    bad_posture_time += detection_interval;
    ESP_LOGW(TAG, "累计不良姿势时间: %d 秒", bad_posture_time);
}
```

## 故障排除

### 1. 编译错误
- 确认 ESP-IDF 版本 >= 5.0
- 检查 esp_camera 组件是否正确安装

### 2. 摄像头初始化失败
- 检查硬件连接
- 确认引脚配置正确
- 检查电源供应

### 3. 性能问题
- 降低图像分辨率
- 增加检测间隔
- 优化内存使用

## 下一步开发

1. **集成真实模型**：参考教程文档中的模型量化和部署步骤
2. **数据收集**：收集真实的坐姿数据用于训练
3. **界面开发**：开发 Web 界面或移动端应用
4. **云端集成**：实现数据上传和分析

## 参考资源

- [ESP-DL 官方文档](https://github.com/espressif/esp-dl)
- [ESP32 Camera 库](https://github.com/espressif/esp32-camera)
- [YOLO11 官方仓库](https://github.com/ultralytics/ultralytics)

## 许可证

本示例代码遵循 ESP-DL 项目的许可证。 