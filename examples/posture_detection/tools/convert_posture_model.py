#!/usr/bin/env python3
"""
ESP-DL 坐姿检测模型转换工具

该脚本用于将 YOLO11n-pose 模型转换为 ESP32 可用的量化模型。

使用方法：
    python convert_posture_model.py --model yolo11n-pose.pt --output posture_model

依赖：
    pip install ultralytics torch onnx ppq
"""

import argparse
import os
import sys
import logging
from pathlib import Path

try:
    import torch
    from ultralytics import YOLO
    import onnx
    from ppq import *
    from ppq.api import *
except ImportError as e:
    print(f"缺少依赖库: {e}")
    print("请安装依赖: pip install ultralytics torch onnx ppq")
    sys.exit(1)

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def download_model(model_name="yolo11n-pose.pt"):
    """下载预训练模型"""
    if not os.path.exists(model_name):
        logger.info(f"下载模型: {model_name}")
        try:
            # YOLO 会自动下载模型
            model = YOLO(model_name)
            logger.info("模型下载完成")
            return model
        except Exception as e:
            logger.error(f"模型下载失败: {e}")
            return None
    else:
        logger.info(f"加载现有模型: {model_name}")
        return YOLO(model_name)

def export_to_onnx(model, output_path, img_size=320):
    """导出模型为ONNX格式"""
    logger.info("导出模型为 ONNX 格式...")
    
    try:
        # 导出ONNX模型
        onnx_path = f"{output_path}.onnx"
        model.export(
            format='onnx',
            imgsz=img_size,
            simplify=True,
            opset=11,  # 使用 ONNX opset 11，兼容性更好
        )
        
        # 重命名为指定路径
        default_name = model.ckpt_path.replace('.pt', '.onnx')
        if os.path.exists(default_name) and default_name != onnx_path:
            os.rename(default_name, onnx_path)
        
        logger.info(f"ONNX 模型已保存到: {onnx_path}")
        return onnx_path
        
    except Exception as e:
        logger.error(f"ONNX 导出失败: {e}")
        return None

def create_calibration_dataset(img_size=320, batch_size=32):
    """创建校准数据集"""
    logger.info("创建校准数据集...")
    
    # 生成随机数据作为校准数据（实际使用时应该用真实图像）
    import numpy as np
    
    def calibration_dataloader():
        for i in range(batch_size):
            # 生成随机图像数据 (1, 3, img_size, img_size)
            # 实际应用中应该使用真实的坐姿图像
            data = np.random.rand(1, 3, img_size, img_size).astype(np.float32)
            yield torch.from_numpy(data)
    
    logger.info(f"校准数据集创建完成 - 批次大小: {batch_size}")
    return calibration_dataloader()

def quantize_model(onnx_path, output_path, target_platform='esp32p4'):
    """量化模型"""
    logger.info(f"开始量化模型，目标平台: {target_platform}")
    
    try:
        # 获取目标平台
        platform = get_target_platform(target_platform)
        
        # 创建量化设置
        setting = QuantizationSettingFactory.espdl_setting()
        
        # 创建校准数据集
        calib_dataloader = create_calibration_dataset()
        
        # 执行量化
        logger.info("执行模型量化...")
        quantized_graph = quantize_onnx_model(
            onnx_import_file=onnx_path,
            calib_dataloader=calib_dataloader,
            calib_steps=32,
            setting=setting,
            platform=platform,
            device='cpu'
        )
        
        # 导出量化后的模型
        export_path = f"{output_path}_quantized"
        export_ppq_graph(
            graph=quantized_graph,
            platform=platform,
            graph_save_to=export_path
        )
        
        logger.info(f"量化模型已保存到: {export_path}")
        return export_path
        
    except Exception as e:
        logger.error(f"模型量化失败: {e}")
        return None

def validate_model(onnx_path):
    """验证ONNX模型"""
    try:
        model = onnx.load(onnx_path)
        onnx.checker.check_model(model)
        logger.info("ONNX 模型验证通过")
        
        # 打印模型信息
        logger.info(f"模型输入: {[inp.name for inp in model.graph.input]}")
        logger.info(f"模型输出: {[out.name for out in model.graph.output]}")
        
        return True
    except Exception as e:
        logger.error(f"模型验证失败: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='ESP-DL 坐姿检测模型转换工具')
    parser.add_argument('--model', default='yolo11n-pose.pt', 
                       help='输入模型路径 (默认: yolo11n-pose.pt)')
    parser.add_argument('--output', default='posture_detection_model',
                       help='输出模型名称 (默认: posture_detection_model)')
    parser.add_argument('--img-size', type=int, default=320,
                       help='图像尺寸 (默认: 320)')
    parser.add_argument('--target', default='esp32p4',
                       choices=['esp32s3', 'esp32p4'],
                       help='目标平台 (默认: esp32p4)')
    
    args = parser.parse_args()
    
    logger.info("=" * 50)
    logger.info("ESP-DL 坐姿检测模型转换工具")
    logger.info("=" * 50)
    logger.info(f"输入模型: {args.model}")
    logger.info(f"输出路径: {args.output}")
    logger.info(f"图像尺寸: {args.img_size}")
    logger.info(f"目标平台: {args.target}")
    
    # 步骤1: 下载/加载模型
    model = download_model(args.model)
    if not model:
        logger.error("模型加载失败")
        return 1
    
    # 步骤2: 导出ONNX模型
    onnx_path = export_to_onnx(model, args.output, args.img_size)
    if not onnx_path:
        logger.error("ONNX 导出失败")
        return 1
    
    # 步骤3: 验证ONNX模型
    if not validate_model(onnx_path):
        logger.error("ONNX 模型验证失败")
        return 1
    
    # 步骤4: 量化模型
    quantized_path = quantize_model(onnx_path, args.output, args.target)
    if not quantized_path:
        logger.error("模型量化失败")
        return 1
    
    logger.info("=" * 50)
    logger.info("转换完成!")
    logger.info(f"ONNX 模型: {onnx_path}")
    logger.info(f"量化模型: {quantized_path}")
    logger.info("=" * 50)
    
    # 后续步骤说明
    print("\n下一步操作:")
    print("1. 将量化后的模型文件复制到 ESP32 项目中")
    print("2. 在 ESP32 代码中加载模型:")
    print(f"   auto model = dl::Model::load_from_file(\"{quantized_path}\");")
    print("3. 编译并烧录程序到 ESP32")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 