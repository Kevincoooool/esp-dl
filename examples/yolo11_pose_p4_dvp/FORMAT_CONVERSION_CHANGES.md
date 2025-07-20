# RGB565 to RGB888 格式转换修改说明

## 问题描述
- 摄像头只支持RGB565格式输出
- 姿态检测需要RGB888格式
- LVGL显示器配置为RGB565格式
- **问题**: RGB565到RGB888转换后画面偏蓝色
- 需要在这些格式之间进行正确的转换

## 解决方案

### 1. 添加格式转换函数

在 `app_main.cpp` 中添加了两个转换函数：

```cpp
// RGB565到RGB888转换函数
void rgb565_to_rgb888(const uint8_t *src, uint8_t *dst, int width, int height);

// RGB888到RGB565转换函数（用于LVGL显示）
void rgb888_to_rgb565(const uint8_t *src, uint8_t *dst, int width, int height);
```

### 2. 修改缓冲区分配

在摄像头任务中增加了多个缓冲区：
- `rgb888_buffer`: RGB888格式的原始图像缓冲区
- `resized_buffer`: 缩放后的RGB888图像缓冲区  
- `display_buffer_rgb888`: 显示用RGB888图像缓冲区
- `display_buffer_rgb565`: 显示用RGB565图像缓冲区

### 3. 数据流处理

```
摄像头(RGB565) -> RGB888转换 -> 姿态检测(RGB888) + 绘制检测结果
                              ↓
                            图像缩放
                              ↓
                        RGB565转换 -> LVGL显示(RGB565)
```

### 4. 关键修改点

1. **摄像头数据获取**：
   ```cpp
   // 格式转换：从RGB565转换为RGB888
   if (camera_format == KSDIY_VIDEO_FMT_RGB565) {
       rgb565_to_rgb888(camera_buffer, rgb888_buffer, camera_width, camera_height);
   }
   ```

2. **图像缩放**：
   ```cpp
   // 使用RGB888格式进行缩放处理
   dl::image::img_t src_img = {
       .data = rgb888_buffer,
       .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888
   };
   ```

3. **显示格式转换**：
   ```cpp
   // 将RGB888转换为RGB565用于LVGL显示
   rgb888_to_rgb565(display_buffer_rgb888, display_buffer_rgb565, TARGET_WIDTH, TARGET_HEIGHT);
   ```

4. **LVGL配置更新**：
   ```cpp
   // 配置图像描述符（使用RGB565格式）
   camera_img_desc.header.cf = LV_COLOR_FORMAT_RGB565;
   camera_img_desc.data_size = TARGET_WIDTH * TARGET_HEIGHT * 2;  // RGB565大小
   ```

### 5. 内存优化

- 为不同格式分配合适大小的缓冲区
- RGB565: `width * height * 2` 字节
- RGB888: `width * height * 3` 字节

### 6. 错误处理

- 检查不支持的像素格式
- 添加缓冲区分配失败检查
- 冻结帧缓冲区的格式自适应分配

## 性能考虑

1. **格式转换开销**：增加了CPU计算开销，但确保了兼容性
2. **内存使用**：增加了额外的缓冲区，但提供了格式灵活性
3. **实时性**：转换操作较快，不影响帧率

## 颜色问题修复

### 1. 字节序问题
修改RGB565读取方式，明确指定小端格式：
```cpp
uint16_t pixel = src_bytes[i * 2] | (src_bytes[i * 2 + 1] << 8);
```

### 2. 颜色通道顺序
尝试BGR顺序来修复颜色偏移：
```cpp
dst_24[i * 3 + 0] = (b << 3) | (b >> 2);  // B: 5位扩展到8位 
dst_24[i * 3 + 1] = (g << 2) | (g >> 4);  // G: 6位扩展到8位  
dst_24[i * 3 + 2] = (r << 3) | (r >> 2);  // R: 5位扩展到8位
```

### 3. RGB565写入优化
在RGB888转RGB565时也明确字节序：
```cpp
dst_bytes[i * 2] = pixel565 & 0xFF;
dst_bytes[i * 2 + 1] = (pixel565 >> 8) & 0xFF;
```

## 测试建议

1. 验证摄像头图像颜色正确显示（不偏蓝）
2. 确认姿态检测功能正常
3. 检查内存使用情况
4. 测试冻结帧功能
5. 验证不同分辨率下的转换效果
6. 检查绘制检测结果时画面不会异常 