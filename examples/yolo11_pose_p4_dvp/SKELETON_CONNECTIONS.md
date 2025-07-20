# 人体骨骼连接说明

本文档详细说明了ESP32-P4姿态检测系统中人体骨骼线条的绘制规则和连接关系。

## 🦴 COCO 17关键点标准

系统基于COCO数据集的17个关键点标准，每个关键点都有固定的索引：

```
0:  nose (鼻子)
1:  left_eye (左眼)
2:  right_eye (右眼)  
3:  left_ear (左耳)
4:  right_ear (右耳)
5:  left_shoulder (左肩)
6:  right_shoulder (右肩)
7:  left_elbow (左肘)
8:  right_elbow (右肘)
9:  left_wrist (左腕)
10: right_wrist (右腕)
11: left_hip (左髋)
12: right_hip (右髋)
13: left_knee (左膝)
14: right_knee (右膝)
15: left_ankle (左踝)
16: right_ankle (右踝)
```

## 🔗 骨骼连接定义

系统定义了17条骨骼连接线，形成完整的人体骨架结构：

### 头部连接 (4条线)
```
nose -> left_eye      (0 -> 1)
nose -> right_eye     (0 -> 2)  
left_eye -> left_ear  (1 -> 3)
right_eye -> right_ear (2 -> 4)
```

### 躯干连接 (4条线)
```
left_shoulder -> right_shoulder   (5 -> 6)   // 肩膀连线
left_shoulder -> left_hip         (5 -> 11)  // 左侧躯干
right_shoulder -> right_hip       (6 -> 12)  // 右侧躯干  
left_hip -> right_hip            (11 -> 12) // 髋部连线
```

### 左臂连接 (2条线)
```
left_shoulder -> left_elbow  (5 -> 7)   // 左上臂
left_elbow -> left_wrist     (7 -> 9)   // 左前臂
```

### 右臂连接 (2条线)
```
right_shoulder -> right_elbow (6 -> 8)   // 右上臂
right_elbow -> right_wrist    (8 -> 10)  // 右前臂
```

### 左腿连接 (2条线)
```
left_hip -> left_knee    (11 -> 13)  // 左大腿
left_knee -> left_ankle  (13 -> 15)  // 左小腿
```

### 右腿连接 (2条线)  
```
right_hip -> right_knee   (12 -> 14)  // 右大腿
right_knee -> right_ankle (14 -> 16)  // 右小腿
```

## 🎨 视觉效果

### 颜色编码
- **红色边框** (255, 0, 0)：人体检测边界框
- **绿色关键点** (0, 255, 0)：17个关键点位置
- **蓝色骨骼线** (0, 100, 255)：骨骼连接线

### 线条属性
- **边框粗细**：2像素
- **骨骼线粗细**：2像素  
- **关键点半径**：3像素

## 🧠 绘制逻辑

### 连接条件
只有当两个关键点都被有效检测到时（坐标 > 0），才会绘制连接线：

```c
if (x1 > 0 && y1 > 0 && x2 > 0 && y2 > 0) {
    draw_line_rgb888(buffer, width, height, x1, y1, x2, y2, 0, 100, 255, 2);
}
```

### 绘制顺序
1. 首先绘制红色边界框
2. 然后绘制绿色关键点
3. 最后绘制蓝色骨骼连接线

这样确保骨骼线条在最上层，不会被其他元素遮挡。

## 📐 人体结构示意图

```
        nose(0)
       /   |   \
   left_eye(1) right_eye(2)
     |             |
  left_ear(3)   right_ear(4)
     
     
  left_shoulder(5) ---- right_shoulder(6)
         |                     |
    left_elbow(7)         right_elbow(8)  
         |                     |
    left_wrist(9)        right_wrist(10)
         
  left_shoulder(5) ---- right_shoulder(6)
         |                     |
    left_hip(11)  ----   right_hip(12)
         |                     |
    left_knee(13)        right_knee(14)
         |                     |
   left_ankle(15)       right_ankle(16)
```

## 🛠️ 技术实现

### Bresenham直线算法
使用经典的Bresenham算法绘制直线，具有以下优点：
- 整数运算，速度快
- 像素连续，无断点
- 支持任意角度直线

### 边界检查
每条线段都进行边界检查，确保：
- 不会访问图像缓冲区外的内存
- 线段完全在图像外时直接跳过
- 部分在图像外的线段正确裁剪

### 性能优化
- 预定义骨骼连接数组，避免运行时计算
- 使用条件编译，可选择开启/关闭骨骼绘制
- 批量绘制，减少函数调用开销

## 🎯 应用场景

### 运动分析
- 姿态评估：检查运动员动作标准性
- 步态分析：观察行走模式
- 康复训练：监控关节活动范围

### 安防监控  
- 行为识别：判断异常动作
- 人员计数：统计通过人数
- 区域监控：检测禁入区域

### 人机交互
- 手势识别：识别特定手势
- 体感游戏：捕捉玩家动作
- 虚拟现实：身体追踪

通过这套完整的骨骼连接系统，您的ESP32-P4设备可以实现专业级的人体姿态分析和可视化！ 