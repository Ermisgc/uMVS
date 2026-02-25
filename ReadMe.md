# MyU3D 项目

## 项目概述

MyU3D 是一个专注于相机模型、标定和立体视觉的C++项目，特别针对水下视觉应用进行了优化。该项目提供了丰富的相机模型实现、标定工具和立体视觉处理功能，适用于计算机视觉、机器人导航和3D重建等领域。

## 目录结构

```
my_u3d/
├── CbtCameraL/         # 相机标定图像目录
├── bin/                # 可执行文件输出目录
│   └── Release/        # 发布版本可执行文件
├── blender_scripts/    # Blender脚本目录
├── build/              # 构建目录
├── inc/                # 头文件目录
│   ├── argparse/       # 命令行参数解析
│   ├── camera/         # 相机相关头文件
│   └── optics/         # 光学相关头文件
├── src/                # 源代码目录
│   ├── camera/         # 相机相关代码
│   └── optics/         # 光学相关代码
├── test/               # 测试代码目录
│   ├── calibrate/      # 标定测试
│   ├── camera/         # 相机测试
│   ├── solver/         # 求解器测试
│   └── stereo/         # 立体视觉测试
├── CMakeLists.txt      # CMake构建配置文件
└── autobuild.sh        # 自动构建脚本
```

## 项目功能

### 相机模型
- 单目相机 (monocamera)
- 单目水下相机 (monouwcamera)
- 双目相机 (binocamera)
- 双目水下相机 (binouwcamera)

### 标定工具
- 单目标定 (calibrateMono)
- 双目标定 (calibrateBino)

### 立体视觉
- 图像校正 (rectify)
- 水下图像校正 (rectifyUW)
- 水下极线几何 (epipolar_uw)

### 光学求解器
- 四次方程求解器 (quartic_solver)
- SVD求解器 (svd)

## 安装说明

### 依赖项
- OpenCV
- Eigen3

### 构建步骤

1. **配置依赖项**
   - 确保已安装OpenCV
   - Eigen3已包含在项目的third_party目录中

2. **构建项目**
   ```bash
   # 使用自动构建脚本
   ./autobuild.sh
   
   # 或手动构建
   mkdir -p build
   cd build
   cmake ..
   cmake --build . --config Release
   ```

## 可执行文件使用方法

### 标定工具

#### 单目标定 (calibrateMono.exe)
```bash
./calibrateMono.exe -i <image_directory> -o <output_calibration_file>
```

#### 双目标定 (calibrateBino.exe)
```bash
./calibrateBino.exe -l <left_image_directory> -r <right_image_directory> -o <output_calibration_file>
```

### 图像校正工具

#### 标准图像校正 (rectify.exe)
```bash
./rectify.exe -l <left_image> -r <right_image> -c <calibration_file> -o <output_directory>
```

#### 水下图像校正 (rectifyUW.exe)
```bash
./rectifyUW.exe -l <left_image> -r <right_image> -c <calibration_file> -o <output_directory>
```

### 测试工具

#### 单目相机测试 (test_monocamera.exe)
```bash
./test_monocamera.exe
```

#### 单目水下相机测试 (test_monouwcamera.exe)
```bash
./test_monouwcamera.exe
```

#### 求解器测试
```bash
# 四次方程求解器测试
./test_quartic_solver.exe

# SVD求解器测试
./test_svd.exe
```

#### 校正垂直误差测试 (test_vertical_error_of_rectify.exe)
```bash
./test_vertical_error_of_rectify.exe -l <left_image> -r <right_image> -c <calibration_file>
```

## 注意事项

1. **依赖项版本**
   - 建议使用OpenCV 4.0或更高版本
   - 项目使用C++17标准，确保编译器支持

2. **标定图像**
   - 标定图像应包含清晰的棋盘格图案
   - 建议使用至少10-15张不同角度和距离的标定图像
   - 水下标定时，确保图像清晰，避免水体浑浊影响标定精度

3. **运行环境**
   - 可执行文件位于`bin/Release/`目录下
   - 运行时确保工作目录正确，或使用绝对路径指定输入/输出文件

4. **性能考虑**
   - 水下视觉处理可能较为耗时，特别是对于高分辨率图像
   - 建议在性能较好的计算机上运行标定和校正操作

5. **文件路径**
   - Windows系统下使用反斜杠(\)作为路径分隔符
   - 命令行参数中的路径应使用绝对路径或相对于可执行文件的路径

## 示例用法

### 单目标定示例
```bash
# 使用CbtCameraL目录中的图像进行单目标定
./calibrateMono.exe -i ../CbtCameraL -o ../calibration/mono_calib.yaml
```

### 双目标定示例
```bash
# 使用左右相机图像进行双目标定
./calibrateBino.exe -l ../CbtCameraL/left -r ../CbtCameraL/right -o ../calibration/bino_calib.yaml
```

### 图像校正示例
```bash
# 校正左右相机图像
./rectify.exe -l ../images/left.jpg -r ../images/right.jpg -c ../calibration/bino_calib.yaml -o ../output/rectified
```

## 故障排除

1. **标定失败**
   - 检查标定图像是否清晰
   - 确保棋盘格图案完整且边缘清晰
   - 尝试使用更多的标定图像

2. **校正结果不理想**
   - 检查标定文件是否正确
   - 确保输入图像与标定图像的分辨率一致
   - 对于水下场景，考虑使用`rectifyUW.exe`进行校正

3. **运行时错误**
   - 检查依赖项是否正确安装
   - 确保输入文件路径正确
   - 查看命令行输出的错误信息

## 许可证

本项目为开源项目，使用MIT许可证。

## 联系方式

如有问题或建议，请联系项目维护者。