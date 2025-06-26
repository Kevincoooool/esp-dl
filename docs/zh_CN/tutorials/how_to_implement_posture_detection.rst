如何实现坐姿检测
=================

:link_to_translation:`en:[English]`

本教程将指导您使用 ESP-DL 实现坐姿检测系统，用于检测学生的头部、脖子、肩膀姿态，判断是否趴下、是否正常坐着、头部是否倾斜等状态。

.. contents::
  :local:
  :depth: 3

项目概述
---------

坐姿检测是一个典型的**姿态估计**任务，我们将通过以下方式实现：

1. **使用现有的姿态估计模型**：基于 YOLO11n-pose 模型进行改进
2. **关键点检测**：检测人体关键点（头顶、眼睛、鼻子、耳朵、肩膀等）
3. **姿态分析**：基于关键点位置计算坐姿状态

支持检测的坐姿状态：
- ✅ **正常坐姿**：头部端正，肩膀平衡
- ❌ **趴桌**：头部过低，接近桌面
- ❌ **歪头**：头部明显倾斜
- ❌ **驼背**：肩膀前倾，姿态不正

准备工作
---------

1. :ref:`安装 ESP_IDF <requirements_esp_idf>`
2. :ref:`安装 ESP_PPQ <requirements_esp_ppq>`
3. **硬件要求**：ESP32-S3 或 ESP32-P4 开发板 + 摄像头模块

方案选择
---------

我们提供两种实现方案：

方案一：使用预训练模型（推荐新手）
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**优点**：
- 无需训练，直接使用
- 快速上手
- 模型已经优化

**缺点**：
- 可能不够精确
- 无法针对特定场景优化

**适用场景**：快速原型验证、学习使用

方案二：自定义训练模型
^^^^^^^^^^^^^^^^^^^^^

**优点**：
- 高精度，针对性强
- 可以根据具体场景优化
- 支持自定义检测类别

**缺点**：
- 需要收集数据
- 训练时间较长
- 需要一定的机器学习基础

**适用场景**：产品级应用、特定场景需求

方案一：使用预训练模型实现坐姿检测
---------------------------------

步骤1：下载预训练模型
^^^^^^^^^^^^^^^^^^^^

我们基于 YOLO11n-pose 模型实现坐姿检测：

.. code-block:: bash

   # 下载预训练的 YOLO11n-pose 模型
   wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt

步骤2：模型转换和量化
^^^^^^^^^^^^^^^^^^^

创建转换脚本 ``convert_posture_model.py``：

.. code-block:: python

   import torch
   from ultralytics import YOLO
   import onnx
   from ppq import *
   from ppq.api import *

   # 1. 加载预训练模型
   model = YOLO('yolo11n-pose.pt')
   
   # 2. 导出为 ONNX 格式
   model.export(format='onnx', imgsz=320, simplify=True)
   
   # 3. 量化模型以适配 ESP32
   def quantize_posture_model():
       # 读取 ONNX 模型
       onnx_model_path = 'yolo11n-pose.onnx'
       
       # 准备校准数据集（使用示例图片）
       calibration_dataset = []
       # 这里应该使用实际的坐姿图片进行校准
       # 可以使用网上的坐姿数据集或自己拍摄的图片
       
       # PPQ 量化设置
       target_platform = get_target_platform('esp32p4')
       quantization_setting = QuantizationSettingFactory.espdl_setting()
       
       # 执行量化
       quantized_model = quantize_onnx_model(
           onnx_import_file=onnx_model_path,
           calib_dataloader=calibration_dataset,
           calib_steps=32,
           setting=quantization_setting,
           platform=target_platform,
           device='cpu'
       )
       
       # 导出量化后的模型
       export_ppq_graph(
           graph=quantized_model,
           platform=target_platform,
           graph_save_to='posture_detection_model'
       )
   
   if __name__ == '__main__':
       quantize_posture_model()

步骤3：实现坐姿检测逻辑
^^^^^^^^^^^^^^^^^^^^^

创建坐姿分析类 ``posture_analyzer.hpp``：

.. code-block:: cpp

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
   };

实现坐姿分析逻辑 ``posture_analyzer.cpp``：

.. code-block:: cpp

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
       if (abs(head_tilt) > head_tilt_threshold) {
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
       float offset_ratio = abs(shoulder_center_x - nose_x) / abs(right_shoulder_x - left_shoulder_x);
       
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

步骤4：主程序实现
^^^^^^^^^^^^^^^^

创建主程序 ``posture_detection_main.cpp``：

.. code-block:: cpp

   #include "esp_log.h"
   #include "esp_camera.h"
   #include "dl_model_base.hpp"
   #include "dl_image_preprocessor.hpp"
   #include "dl_pose_yolo11_postprocessor.hpp"
   #include "posture_analyzer.hpp"
   #include <vector>

   static const char *TAG = "PostureDetection";

   // 相机配置
   camera_config_t camera_config = {
       .pin_pwdn = -1,
       .pin_reset = -1,
       .pin_xclk = 4,
       .pin_sscb_sda = 18,
       .pin_sscb_scl = 23,
       .pin_d7 = 36,
       .pin_d6 = 37,
       .pin_d5 = 38,
       .pin_d4 = 39,
       .pin_d3 = 35,
       .pin_d2 = 14,
       .pin_d1 = 13,
       .pin_d0 = 34,
       .pin_vsync = 5,
       .pin_href = 27,
       .pin_pclk = 25,
       .xclk_freq_hz = 20000000,
       .ledc_timer = LEDC_TIMER_0,
       .ledc_channel = LEDC_CHANNEL_0,
       .pixel_format = PIXFORMAT_RGB565,
       .frame_size = FRAMESIZE_QVGA, // 320x240
       .jpeg_quality = 10,
       .fb_count = 1
   };

   extern "C" void app_main() {
       ESP_LOGI(TAG, "坐姿检测系统启动");
       
       // 1. 初始化相机
       esp_err_t err = esp_camera_init(&camera_config);
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "相机初始化失败");
           return;
       }
       
       // 2. 加载姿态检测模型
       auto model = dl::Model::load_from_file("posture_detection_model");
       if (!model) {
           ESP_LOGE(TAG, "模型加载失败");
           return;
       }
       
       // 3. 初始化图像预处理器
       dl::image::ImagePreprocessor preprocessor(320, 240, 3);
       
       // 4. 初始化后处理器
       dl::detect::PoseYOLO11PostProcessor postprocessor(0.25, 0.45, 17); // 17个关键点
       
       // 5. 初始化坐姿分析器
       PostureAnalyzer posture_analyzer;
       
       ESP_LOGI(TAG, "初始化完成，开始检测");
       
       while (true) {
           // 6. 获取摄像头图像
           camera_fb_t *fb = esp_camera_fb_get();
           if (!fb) {
               ESP_LOGE(TAG, "获取图像失败");
               continue;
           }
           
           // 7. 图像预处理
           auto input_tensor = preprocessor.preprocess(fb->buf, fb->len);
           if (!input_tensor) {
               ESP_LOGE(TAG, "图像预处理失败");
               esp_camera_fb_return(fb);
               continue;
           }
           
           // 8. 模型推理
           auto output = model->run(input_tensor);
           if (!output) {
               ESP_LOGE(TAG, "模型推理失败");
               esp_camera_fb_return(fb);
               continue;
           }
           
           // 9. 后处理 - 提取关键点
           auto detections = postprocessor.process(output);
           
           // 10. 坐姿分析
           if (!detections.empty()) {
               // 获取第一个检测到的人体关键点
               auto& detection = detections[0];
               std::vector<Keypoint> keypoints;
               
               // 转换为 Keypoint 格式
               for (int i = 0; i < detection.keypoints.size(); i += 3) {
                   Keypoint kp;
                   kp.x = detection.keypoints[i];
                   kp.y = detection.keypoints[i + 1];
                   kp.confidence = detection.keypoints[i + 2];
                   keypoints.push_back(kp);
               }
               
               // 分析坐姿
               PostureState state = posture_analyzer.analyzePosture(keypoints);
               const char* description = posture_analyzer.getPostureDescription(state);
               
               // 输出结果
               if (state == NORMAL_SITTING) {
                   ESP_LOGI(TAG, "✅ %s", description);
               } else {
                   ESP_LOGW(TAG, "❌ %s", description);
                   
                   // 可以在这里添加警告机制
                   // 例如：LED 指示灯、蜂鸣器等
               }
               
               // 详细信息输出
               float head_tilt = posture_analyzer.calculateHeadTilt(keypoints);
               ESP_LOGI(TAG, "头部倾斜角度: %.1f°", head_tilt);
           } else {
               ESP_LOGI(TAG, "未检测到人体");
           }
           
           // 11. 清理资源
           esp_camera_fb_return(fb);
           
           // 12. 延时
           vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒检测一次
       }
   }

方案二：自定义训练坐姿检测模型
-----------------------------

如果您需要更高的精度或特定场景的优化，可以训练自定义模型。

步骤1：数据收集
^^^^^^^^^^^^^^

**收集坐姿图像数据**：

1. **正常坐姿图片**：200-500张
2. **趴桌图片**：200-500张  
3. **歪头图片**：200-500张
4. **驼背图片**：200-500张

**数据收集建议**：
- 不同光照条件下拍摄
- 不同角度和距离
- 不同的人（年龄、体型）
- 使用与实际部署相同的摄像头

步骤2：数据标注
^^^^^^^^^^^^^^

使用 `labelme <https://github.com/wkentaro/labelme>`_ 或 `CVAT <https://cvat.org/>`_ 进行关键点标注：

.. code-block:: bash

   # 安装 labelme
   pip install labelme
   
   # 启动标注工具
   labelme

**标注要求**：
- 标注17个COCO关键点
- 重点关注头部、肩膀关键点的准确性
- 为每个状态创建不同的类别标签

步骤3：模型训练
^^^^^^^^^^^^^^

使用 YOLOv8/v11 进行训练：

.. code-block:: python

   from ultralytics import YOLO
   import yaml

   # 创建数据配置文件 posture_dataset.yaml
   dataset_config = {
       'path': './posture_dataset',
       'train': 'images/train',
       'val': 'images/val',
       'names': {
           0: 'person'  # 只检测人体，关键点用于姿态分析
       },
       'kpt_shape': [17, 3]  # 17个关键点，每个点3个值(x,y,v)
   }

   with open('posture_dataset.yaml', 'w') as f:
       yaml.dump(dataset_config, f)

   # 训练模型
   model = YOLO('yolo11n-pose.pt')  # 加载预训练模型
   
   # 开始训练
   results = model.train(
       data='posture_dataset.yaml',
       epochs=100,
       imgsz=320,
       batch=16,
       device='cuda'  # 如果有GPU
   )

步骤4：模型验证和优化
^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # 验证模型性能
   metrics = model.val()
   print(f"mAP50: {metrics.box.map50}")
   print(f"关键点准确度: {metrics.pose.map50}")

   # 测试推理速度
   import time
   results = model.predict('test_image.jpg', verbose=False)
   
   # 如果精度不够，可以：
   # 1. 增加训练数据
   # 2. 调整模型参数
   # 3. 使用数据增强
   # 4. 尝试更大的模型（如 yolo11s-pose）

部署配置
---------

步骤1：硬件连接
^^^^^^^^^^^^^^

**ESP32-S3 + OV2640 摄像头模块**：

.. code-block:: c

   // 相机引脚配置（根据实际硬件调整）
   #define CAM_PIN_PWDN    -1  // Power down not used
   #define CAM_PIN_RESET   -1  // Software reset
   #define CAM_PIN_XCLK    4   // External clock
   #define CAM_PIN_SIOD    18  // SDA
   #define CAM_PIN_SIOC    23  // SCL
   #define CAM_PIN_D7      36
   #define CAM_PIN_D6      37
   #define CAM_PIN_D5      38
   #define CAM_PIN_D4      39
   #define CAM_PIN_D3      35
   #define CAM_PIN_D2      14
   #define CAM_PIN_D1      13
   #define CAM_PIN_D0      34
   #define CAM_PIN_VSYNC   5
   #define CAM_PIN_HREF    27
   #define CAM_PIN_PCLK    25

步骤2：编译和烧录
^^^^^^^^^^^^^^^

.. code-block:: bash

   # 设置 IDF 环境（在 F:\Espressif\frameworks\esp-idf-v5.4.1 目录）
   cd F:\Espressif\frameworks\esp-idf-v5.4.1
   .\export.ps1
   
   # 返回项目目录
   cd F:\GitHub\esp-dl\examples\posture_detection
   
   # 配置项目
   idf.py menuconfig
   
   # 编译项目
   idf.py build
   
   # 烧录到设备
   idf.py flash monitor

性能优化建议
-----------

1. **模型优化**：
   - 使用较小的输入尺寸（320x240）
   - 启用模型量化（INT8）
   - 删除不必要的关键点检测

2. **算法优化**：
   - 实现帧间跟踪，避免每帧都做完整检测
   - 使用运动检测，静止时降低检测频率
   - 多帧平滑处理，避免误判

3. **硬件优化**：
   - 使用 ESP32-P4 获得更好的AI性能
   - 优化相机参数，提高图像质量
   - 合理设置系统时钟频率

故障排除
---------

**常见问题**：

1. **模型加载失败**：
   - 检查模型文件路径
   - 确认模型格式正确
   - 检查内存是否足够

2. **检测精度不高**：
   - 调整置信度阈值
   - 改善光照条件
   - 重新训练模型

3. **程序运行缓慢**：
   - 降低图像分辨率
   - 优化模型大小
   - 减少检测频率

4. **相机初始化失败**：
   - 检查硬件连接
   - 确认引脚配置
   - 检查电源供应

总结
----

通过本教程，您可以：

1. **快速上手**：使用方案一快速实现基本的坐姿检测
2. **深度定制**：使用方案二训练专门的坐姿检测模型
3. **产品化应用**：优化性能，实现实时检测

**推荐学习路径**：
1. 先使用方案一验证硬件和基本功能
2. 收集实际场景数据
3. 使用方案二训练定制模型
4. 持续优化和改进

**下一步扩展**：
- 添加多人检测支持
- 实现检测结果的可视化
- 集成云端数据分析
- 添加移动端监控界面

如有问题，可以参考 ESP-DL 的其他教程或在社区中寻求帮助。 