# ESP32-P4 AI坐姿检测系统

基于ESP32-P4和YOLO11n-pose模型的实时坐姿检测系统。

## 功能特性

- ✅ **实时坐姿检测**: 使用YOLO11n-pose模型进行人体关键点检测
- ✅ **多种坐姿识别**: 正常坐姿、趴桌、歪头、驼背、前倾等
- ✅ **可视化界面**: LVGL界面显示摄像头画面和检测结果
- ✅ **性能优化**: 针对ESP32-P4优化的推理性能
- ✅ **实时反馈**: 检测到不良坐姿时提供即时提醒

## 硬件要求

- ESP32-P4开发板
- SC2336摄像头模块
- LCD显示屏（ST7703或GC9503）
- 16MB Flash (推荐)
- PSRAM (必需)

## 软件要求

- ESP-IDF v5.4+
- ESP-DL框架
- LVGL v9.0+

## 编译说明

1. 配置ESP-IDF环境:
```bash
cd F:\Espressif\frameworks\esp-idf-v5.4.1
export.bat
```

2. 编译项目:
```bash
cd examples/lcd_camera
idf.py build
```

3. 烧录到设备:
```bash
idf.py -p COM15 flash monitor
```

## 系统架构

```
摄像头 → RGB565格式 → YOLO11n-pose → 关键点提取 → 坐姿分析 → UI显示
   ↓                      ↓              ↓           ↓
V4L2接口              AI推理引擎      几何计算    LVGL界面
```

## 坐姿检测算法

### 关键点检测
使用COCO格式的17个人体关键点:
- 面部: 鼻子、眼睛、耳朵
- 躯干: 肩膀、臀部
- 肢体: 肘关节、腕关节、膝关节、踝关节

### 坐姿分析
1. **头部倾斜**: 计算耳朵连线与水平线的角度
2. **脊柱弯曲**: 分析鼻子-肩膀-臀部的角度关系
3. **趴桌检测**: 判断头部相对肩膀的高度位置
4. **肩膀平衡**: 检测两肩高度差异

## 性能指标

- **推理时间**: ~200-500ms (ESP32-P4 @ 400MHz)
- **检测频率**: 每10帧检测一次 (~3Hz)
- **准确率**: >85% (在标准办公环境下)
- **内存占用**: ~8MB PSRAM

## 配置参数

可在代码中调整的检测阈值:
```c
head_tilt_threshold = 20.0f;    // 头部倾斜角度阈值 (度)
lying_head_threshold = 0.7f;    // 趴桌检测阈值
hunch_angle_threshold = 25.0f;  // 驼背角度阈值 (度)
min_confidence = 0.4f;          // 最小检测置信度
```

## 项目结构

```
examples/lcd_camera/
├── main/
│   ├── app_main.c                 # 主程序
│   ├── app_video.c/h              # 摄像头接口
│   ├── ksdiy_lvgl_port.c/h        # LVGL显示接口
│   ├── posture_analyzer.hpp/cpp   # 坐姿分析器
│   ├── posture_analyzer_wrapper.cpp # C包装器
│   └── CMakeLists.txt
├── partitions.csv                 # 分区表
├── sdkconfig.defaults            # 默认配置
└── CMakeLists.txt               # 项目配置
```

## 故障排除

### 常见问题

1. **编译错误**: 检查ESP-DL和COCO Pose模型是否正确包含
2. **内存不足**: 确保启用PSRAM并正确配置
3. **摄像头无法初始化**: 检查摄像头连接和配置
4. **AI推理失败**: 确保模型文件存在且格式正确

### 调试日志

启用调试日志:
```bash
idf.py menuconfig
# Component config → Log output → Default log verbosity → Debug
```

## 扩展功能

可考虑添加的功能:
- [ ] 声音提醒
- [ ] 坐姿统计记录
- [ ] 网络数据上传
- [ ] 多人检测支持
- [ ] 自定义检测阈值界面

## 许可证

本项目遵循MIT许可证。

## 贡献

欢迎提交issue和pull request来改进这个项目。 