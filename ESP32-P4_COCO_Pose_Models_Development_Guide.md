# ESP32-P4 COCO姿态检测模型开发指南

## 概述

本文档详细记录了为ESP32-P4芯片创建多尺寸COCO姿态检测模型的完整过程，包括从YOLO11模型导出、量化转换、代码集成到最终部署的所有步骤。

## 项目背景

### 目标
- 基于现有的COCO检测模型创建COCO姿态检测模型
- 生成多个尺寸规格的模型以适应不同性能需求
- 确保模型能在ESP32-P4芯片上正常运行

### 技术栈
- **硬件平台**: ESP32-P4
- **深度学习框架**: ESP-DL
- **模型**: YOLO11n-pose
- **量化工具**: ESP-PPQ
- **开发环境**: ESP-IDF v5.4.1

## 开发流程

### 第一阶段：320尺寸模型创建

#### 1. 代码框架准备

首先需要在ESP-DL框架中添加对新模型类型的支持：

**修改头文件 (`models/coco_pose/coco_pose.hpp`)**
```cpp
// 添加新的模型类型枚举
typedef enum {
    YOLO11N_POSE_S8_V1 = 0,
    YOLO11N_POSE_S8_V2,
    YOLO11N_POSE_320_S8_V2,  // 新增320尺寸模型
    YOLO11N_POSE_256_P4,     // 新增256尺寸P4模型
    YOLO11N_POSE_224_P4,     // 新增224尺寸P4模型
} coco_pose_model_type_t;
```

**修改实现文件 (`models/coco_pose/coco_pose.cpp`)**
```cpp
// 在构造函数中添加新模型的处理逻辑
switch (model_type) {
    case YOLO11N_POSE_320_S8_V2:
        this->model_name = "YOLO11N_POSE_320_S8_V2";
        this->score_threshold = 0.3;
        this->nms_threshold = 0.45;
        this->model_height = 320;
        this->model_width = 320;
        break;
    // ... 其他case分支
}
```

#### 2. ONNX模型导出

**创建导出脚本 (`models/coco_pose/models/export_onnx.py`)**

```python
from ultralytics import YOLO

def export_yolo_models():
    # 加载预训练模型
    model = YOLO("yolo11n-pose.pt")
    
    # 导出640尺寸模型
    model.export(
        format="onnx",
        imgsz=640,
        simplify=True,
        opset=11
    )
    
    # 导出320尺寸模型
    model.export(
        format="onnx", 
        imgsz=320,
        simplify=True,
        opset=11
    )

if __name__ == "__main__":
    export_yolo_models()
```

**为什么这样导出？**
- `imgsz`参数控制输入图像尺寸，直接影响模型的输入层维度
- `simplify=True`优化ONNX图结构，减少冗余节点
- `opset=11`确保与ESP-PPQ转换工具的兼容性

#### 3. 模型量化转换

**创建量化脚本 (`models/coco_pose/models/convert_320_to_espdl.py`)**

```python
from esp_ppq import espdl_quantize_onnx

def convert_to_espdl():
    espdl_quantize_onnx(
        onnx_model_path="yolo11n-pose-320.onnx",
        espdl_model_save_path="yolo11n-pose-320.espdl",
        calibration_dataset_path="calib_yolo11n-pose",
        target="esp32s3",  # 或 "esp32p4"
        num_of_bits=8,
        algorithm="kl",
        calibration_method="percentage", 
        percentile=99.99,
        batch_size=32
    )
```

**量化参数解释：**
- `target`: 目标芯片，决定量化策略和指令集优化
- `num_of_bits=8`: 8位量化，平衡精度和性能
- `algorithm="kl"`: KL散度量化算法，提供更好的精度保持
- `calibration_method="percentage"`: 使用百分位数确定量化范围
- `percentile=99.99`: 保留99.99%的数据范围，避免异常值影响

### 第二阶段：240和200尺寸模型创建

#### 1. 批量ONNX导出

**创建批量导出脚本 (`models/coco_pose/models/export_all_sizes.py`)**

```python
from ultralytics import YOLO

def export_multiple_sizes():
    model = YOLO("yolo11n-pose.pt")
    
    sizes = [640, 320, 240, 200]
    
    for size in sizes:
        print(f"导出 {size} 尺寸模型...")
        model.export(
            format="onnx",
            imgsz=size,
            simplify=True, 
            opset=11
        )
```

**注意：YOLO自动调整机制**
- 240 → 256 (自动调整到32的倍数)
- 200 → 224 (自动调整到32的倍数)

这是因为YOLO模型要求输入尺寸必须是32的倍数，以确保特征图在下采样过程中维度正确。

#### 2. P4专用量化转换

**256尺寸转换脚本 (`models/coco_pose/models/convert_256_p4_final.py`)**

```python
def convert_256_p4():
    espdl_quantize_onnx(
        onnx_model_path="yolo11n-pose-256.onnx",
        espdl_model_save_path="yolo11n-pose-256-p4.espdl", 
        calibration_dataset_path="calib_yolo11n-pose",
        target="esp32p4",  # P4专用配置
        num_of_bits=8,
        algorithm="kl",
        calibration_method="percentage",
        percentile=99.99,
        batch_size=32
    )
```

### 第三阶段：代码集成和配置

#### 1. 编译配置更新

**修改CMakeLists.txt**
```cmake
# 根据配置选择模型文件
if(CONFIG_YOLO11N_POSE_224_P4_FLASH)
    set(model_name "yolo11n-pose-224-p4")
elseif(CONFIG_YOLO11N_POSE_256_P4_FLASH) 
    set(model_name "yolo11n-pose-256-p4")
elseif(CONFIG_YOLO11N_POSE_320_P4_FLASH)
    set(model_name "yolo11n-pose-320-p4") 
else()
    set(model_name "yolo11n-pose-s8_v2")
endif()

# 嵌入模型文件到固件
target_add_binary_data(${COMPONENT_LIB} 
    "${CMAKE_CURRENT_SOURCE_DIR}/models/p4/${model_name}.espdl" 
    BINARY
)
```

#### 2. Kconfig配置文件

**添加用户配置选项 (`models/coco_pose/Kconfig`)**
```
menu "COCO Pose Model Configuration"
    choice COCO_POSE_MODEL_SELECTION
        prompt "Select COCO Pose Model"
        default YOLO11N_POSE_S8_V2_FLASH
        
        config YOLO11N_POSE_224_P4_FLASH
            bool "YOLO11N Pose 224 P4 (Fast)"
            
        config YOLO11N_POSE_256_P4_FLASH  
            bool "YOLO11N Pose 256 P4 (Balanced)"
            
        config YOLO11N_POSE_320_P4_FLASH
            bool "YOLO11N Pose 320 P4 (Accurate)"
    endchoice
endmenu
```

## 技术原理详解

### 为什么ESP-DL能调用这些模型？

#### 1. **统一的模型格式**
ESP-DL使用自定义的`.espdl`格式，这种格式：
- 针对ESP芯片的内存布局优化
- 包含量化参数和权重数据
- 支持流式加载，适应ESP芯片的内存限制

#### 2. **量化适配**
```cpp
// ESP-DL内部的量化数据处理
class TensorBase {
    int8_t *data;      // 量化后的权重数据
    float scale;       // 量化比例因子  
    int zero_point;    // 零点偏移
};
```

#### 3. **算子映射**
ESP-DL将ONNX算子映射到ESP芯片优化的实现：
- Conv2D → ESP-DL optimized convolution
- Sigmoid → ESP-DL activation functions
- Reshape → ESP-DL tensor operations

#### 4. **内存管理**
```cpp
// 激活缓冲区管理
class ActivateBuffer {
    void *malloc_32_aligned(size_t size);
    void free_32_aligned(void *ptr);
};
```

### 模型加载机制

#### 1. **二进制嵌入**
```cpp
// CMake将.espdl文件编译为二进制数据
extern const uint8_t model_data_start[] asm("_binary_model_espdl_start");
extern const uint8_t model_data_end[] asm("_binary_model_espdl_end");
```

#### 2. **FlatBuffers解析**
```cpp
// ESP-DL使用FlatBuffers序列化格式
fbs::FBSLoader loader;
loader.load(model_data_start, model_data_end - model_data_start);
```

#### 3. **动态图构建**
```cpp
// 运行时构建计算图
class Model {
    std::vector<layer::Layer *> layers;
    void forward(Tensor &input);
};
```

## 性能优化考虑

### 1. **尺寸选择策略**
- **224×224**: 最快推理速度，适合实时应用
- **256×256**: 平衡精度和性能
- **320×320**: 最高精度，适合对准确度要求高的场景

### 2. **内存使用**
```
模型文件大小：
- ONNX: ~11MB (未量化)
- ESPDL: ~3MB (8位量化)

运行时内存：
- 224尺寸: ~2MB激活内存
- 256尺寸: ~2.5MB激活内存  
- 320尺寸: ~3.2MB激活内存
```

### 3. **推理性能**
基于ESP32-P4的理论性能估算：
- 224尺寸: ~150ms/frame
- 256尺寸: ~200ms/frame
- 320尺寸: ~300ms/frame

## 部署验证

### 1. **编译验证**
```bash
# 设置目标芯片
idf.py set-target esp32p4

# 配置模型选择
idf.py menuconfig

# 编译项目
idf.py build
```

### 2. **固件刷写**
```bash
# 刷写固件到设备
idf.py flash

# 监控输出
idf.py monitor
```

### 3. **运行验证**
成功的输出应该包含：
```
I (xxx) COCO_POSE: Model loaded: YOLO11N_POSE_256_P4
I (xxx) COCO_POSE: Input shape: [1, 3, 256, 256]
I (xxx) COCO_POSE: Model ready for inference
```

## 总结

通过以上步骤，我们成功创建了三个不同尺寸的ESP32-P4 COCO姿态检测模型：

1. **模型导出**: 使用Ultralytics YOLO导出标准ONNX格式
2. **量化转换**: 使用ESP-PPQ将ONNX转换为ESP-DL格式
3. **代码集成**: 在ESP-DL框架中添加模型支持和配置选项
4. **编译部署**: 通过ESP-IDF构建系统编译和刷写固件

这种方法确保了模型能够在ESP32-P4芯片上高效运行，同时提供了灵活的配置选项来平衡性能和精度需求。

## 附录

### 相关文件列表
```
models/coco_pose/
├── coco_pose.hpp              # 模型类定义
├── coco_pose.cpp              # 模型实现
├── CMakeLists.txt             # 编译配置
├── Kconfig                    # 用户配置选项
└── models/
    ├── export_onnx.py         # ONNX导出脚本
    ├── export_all_sizes.py    # 批量导出脚本
    ├── convert_256_p4_final.py # 256尺寸转换脚本
    ├── convert_224_p4_final.py # 224尺寸转换脚本
    └── p4/
        ├── yolo11n-pose-224-p4.espdl
        ├── yolo11n-pose-256-p4.espdl
        └── yolo11n-pose-320-p4.espdl
```

### 开发环境要求
- ESP-IDF v5.4.1+
- Python 3.8+
- ultralytics
- esp-ppq
- onnx
- onnxsim 