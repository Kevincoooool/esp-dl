# 显示模式控制配置说明

本项目新增了多个宏定义来控制摄像头画面显示和边框绘制行为，您可以根据需要修改这些宏来实现不同的显示效果。

## 🎛️ 宏定义说明

### 1. CONTINUOUS_REFRESH_MODE
```c
#define CONTINUOUS_REFRESH_MODE     1    // 1: 持续刷新摄像头画面, 0: 检测到人体后暂停刷新
```

**作用**：控制摄像头画面是否持续刷新
- `1` (默认)：摄像头画面持续实时更新，类似视频流
- `0`：检测到人体后暂停画面刷新，保持静止状态

### 2. FREEZE_ON_DETECTION
```c
#define FREEZE_ON_DETECTION         1    // 1: 检测到人体时冻结画面保持边框, 0: 边框会被新帧覆盖
```

**作用**：控制检测到人体时是否冻结画面
- `1` (默认)：检测到人体时冻结当前帧（包含边框和关键点），保持显示一段时间
- `0`：边框会被后续的新帧覆盖，可能导致边框闪烁

### 3. DETECTION_HOLD_TIME_MS
```c
#define DETECTION_HOLD_TIME_MS      3000 // 检测到人体后保持画面冻结的时间(毫秒)
```

**作用**：设置画面冻结持续时间
- 默认3000毫秒（3秒）
- 只在 `FREEZE_ON_DETECTION = 1` 时生效
- 超过此时间后自动恢复正常刷新

### 4. SHOW_BBOX_ONLY_ON_DETECTION
```c
#define SHOW_BBOX_ONLY_ON_DETECTION 0    // 1: 只在检测到人体时显示边框, 0: 实时显示检测结果
```

**作用**：控制边框显示策略
- `0` (默认)：实时显示检测结果，有检测就画边框
- `1`：只在明确检测到人体时才显示边框

### 5. DRAW_SKELETON_LINES
```c
#define DRAW_SKELETON_LINES         1    // 1: 绘制人体骨骼线条, 0: 只显示关键点
```

**作用**：控制是否绘制人体骨骼连接线
- `1` (默认)：绘制蓝色骨骼线条连接各个关键点，形成完整人体结构
- `0`：只显示绿色关键点，不绘制连接线

## 🎯 推荐配置组合

### 组合1：实时流模式（默认）
```c
#define CONTINUOUS_REFRESH_MODE     1
#define FREEZE_ON_DETECTION         1
#define DETECTION_HOLD_TIME_MS      3000
#define SHOW_BBOX_ONLY_ON_DETECTION 0
#define DRAW_SKELETON_LINES         1
```
**效果**：摄像头画面持续更新，检测到人体时冻结3秒显示边框、关键点和骨骼线条

### 组合2：抓拍模式
```c
#define CONTINUOUS_REFRESH_MODE     0
#define FREEZE_ON_DETECTION         1
#define DETECTION_HOLD_TIME_MS      5000
#define SHOW_BBOX_ONLY_ON_DETECTION 1
#define DRAW_SKELETON_LINES         1
```
**效果**：检测到人体后停止刷新并冻结画面5秒，显示完整骨骼结构，类似拍照效果

### 组合3：实时检测模式
```c
#define CONTINUOUS_REFRESH_MODE     1
#define FREEZE_ON_DETECTION         0
#define DETECTION_HOLD_TIME_MS      1000
#define SHOW_BBOX_ONLY_ON_DETECTION 0
#define DRAW_SKELETON_LINES         1
```
**效果**：连续实时显示，边框和骨骼线条跟随检测结果实时更新

### 组合4：精确检测模式
```c
#define CONTINUOUS_REFRESH_MODE     1
#define FREEZE_ON_DETECTION         1
#define DETECTION_HOLD_TIME_MS      2000
#define SHOW_BBOX_ONLY_ON_DETECTION 1
#define DRAW_SKELETON_LINES         1
```
**效果**：只在确实检测到人体时才显示边框和骨骼线条并冻结2秒

### 组合5：简洁模式
```c
#define CONTINUOUS_REFRESH_MODE     1
#define FREEZE_ON_DETECTION         1
#define DETECTION_HOLD_TIME_MS      2000
#define SHOW_BBOX_ONLY_ON_DETECTION 0
#define DRAW_SKELETON_LINES         0
```
**效果**：只显示边框和关键点，不绘制骨骼线条，界面更简洁

## 🔧 使用方法

1. 打开 `main/app_main.cpp` 文件
2. 找到宏定义部分（约在第20-25行）
3. 根据需要修改宏定义的值
4. 重新编译并烧录固件：
   ```bash
   idf.py build
   idf.py flash
   ```

## 💡 技术细节

- **冻结帧缓冲区**：当启用冻结模式时，系统会在PSRAM中分配额外的缓冲区来保存带边框的图像
- **时间控制**：使用ESP32的高精度定时器来控制冻结时间
- **内存管理**：冻结缓冲区只在首次检测到人体时分配，避免重复分配
- **线程安全**：所有显示操作都通过LVGL锁机制保证线程安全
- **骨骼连接算法**：基于COCO 17关键点标准，定义了17条骨骼连接线
- **直线绘制**：使用Bresenham算法高效绘制抗锯齿直线
- **颜色编码**：红色边框、绿色关键点、蓝色骨骼线条，便于区分

## 🚀 性能影响

- **内存使用**：启用冻结模式会额外使用320KB PSRAM
- **CPU使用**：冻结模式下CPU负载略有降低（减少显示更新）
- **响应速度**：不同模式对检测响应速度影响很小

根据您的应用场景选择合适的配置组合，可以获得最佳的用户体验！ 