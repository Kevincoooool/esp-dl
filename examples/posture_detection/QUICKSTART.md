# 坐姿检测快速开始指南

本指南将帮助您在30分钟内搭建一个基本的坐姿检测系统。

## 🚀 第一阶段：快速体验（10分钟）

使用演示数据快速验证硬件和框架。

### 1. 硬件准备
- ESP32-S3 开发板
- OV2640 摄像头模块  
- USB数据线

### 2. 软件环境
```bash
# 设置ESP-IDF环境
cd F:\Espressif\frameworks\esp-idf-v5.4.1
.\export.ps1
```

### 3. 编译运行
```bash
cd F:\GitHub\esp-dl\examples\posture_detection

# 编译
idf.py build

# 烧录
idf.py flash monitor
```

### 4. 查看结果
串口输出将显示：
```
I (1234) PostureDetection: ✅ 检测结果: 正常坐姿
I (3456) PostureDetection: ❌ 检测结果: 趴桌  
I (5678) PostureDetection: ❌ 检测结果: 歪头
```

## 🎯 第二阶段：模型集成（15分钟）

集成真实的YOLO姿态检测模型。

### 1. 安装Python依赖
```bash
pip install ultralytics torch onnx ppq
```

### 2. 转换模型
```bash
cd examples/posture_detection/tools

# 自动下载并转换模型
python convert_posture_model.py --target esp32s3
```

### 3. 更新代码
修改 `main/app_main.cpp`，替换演示数据为真实模型推理：

```cpp
// 在app_main()中添加模型加载
auto model = dl::Model::load_from_file("posture_detection_model_quantized");
if (!model) {
    ESP_LOGE(TAG, "模型加载失败");
    return;
}
```

## 🔧 第三阶段：性能优化（5分钟）

调整参数以适应您的具体场景。

### 1. 调整检测阈值
```cpp
// 根据实际情况调整
analyzer.setHeadTiltThreshold(10.0);    // 更敏感的歪头检测  
analyzer.setLyingHeightRatio(0.2);      // 更严格的趴桌检测
```

### 2. 优化检测频率
```cpp
// 根据性能需求调整
vTaskDelay(pdMS_TO_TICKS(500));  // 0.5秒检测一次，更实时
```

### 3. 添加功能扩展
```cpp
// 添加警告机制
if (state != NORMAL_SITTING) {
    // LED闪烁警告
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_PIN, 0);
    
    // 记录不良姿势时间
    bad_posture_count++;
    ESP_LOGW(TAG, "不良姿势次数: %d", bad_posture_count);
}
```

## 🎓 进阶开发

### 自定义训练数据
1. **数据收集**：使用摄像头拍摄不同坐姿
2. **数据标注**：使用labelme标注关键点
3. **模型训练**：基于YOLO11n-pose微调
4. **模型部署**：重新量化并部署

### 系统集成
1. **Web界面**：添加实时监控界面
2. **数据分析**：统计坐姿数据
3. **云端同步**：实现多设备监控
4. **移动端**：开发配套APP

## 📋 常见问题

### Q1: 摄像头初始化失败
**解决**：检查引脚连接，确认相机模块兼容性

### Q2: 检测不准确
**解决**：
- 改善光照条件
- 调整摄像头角度
- 重新校准检测阈值

### Q3: 系统运行缓慢
**解决**：
- 降低检测频率
- 减小图像分辨率
- 使用ESP32-P4获得更好性能

### Q4: 内存不足
**解决**：
- 启用PSRAM
- 优化模型大小
- 减少缓冲区数量

## 🌟 功能扩展建议

### 基础版本
- [x] 基本坐姿检测
- [x] 实时警告提醒
- [x] 串口状态输出

### 进阶版本
- [ ] Web监控界面
- [ ] 数据统计分析
- [ ] 多人检测支持
- [ ] 声音提醒功能

### 专业版本
- [ ] 云端数据同步
- [ ] AI学习优化
- [ ] 移动端APP
- [ ] 健康报告生成

## 📞 技术支持

遇到问题？

1. **查看日志**：通过串口监控详细错误信息
2. **检查硬件**：确认所有连接正确
3. **参考文档**：阅读ESP-DL官方文档
4. **社区求助**：在ESP32论坛发布问题

## 🎉 恭喜！

您已经成功搭建了一个基于ESP32的坐姿检测系统！

现在您可以：
- 实时监控坐姿状态
- 及时发现不良坐姿
- 根据需求自定义功能
- 继续深入学习AI模型部署

**下一步建议**：
1. 尝试收集自己的训练数据
2. 探索更多ESP-DL功能
3. 分享您的项目经验
4. 贡献代码改进