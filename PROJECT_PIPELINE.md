# CamCalib 软件使用手册与安装教程

## 1. 软件简介

CamCalib 是一个基于 C++17、OpenCV 和 Eigen 的相机/投影仪标定程序，当前支持：

- 圆点标定板特征检测与亚像素圆心提取。
- 白底黑圆和黑底白圆两种标定板极性。
- OpenCV 相机内参、畸变和各位姿外参标定。
- 三频四步相移法绝对相位解算。
- 伪相机法投影仪内参、畸变和各位姿外参标定。
- 相机与投影仪逐位姿重投影误差评价。
- 圆点检测结果和绝对相位图调试输出。

程序默认依次执行：

```text
相机标定
→ 投影仪相位解算
→ 相机圆心与绝对相位匹配
→ 投影仪标定
→ 重投影误差评价
```

## 2. 系统要求

推荐环境：

- Linux（推荐 Ubuntu 20.04 或更新版本）
- CMake 3.16 或更新版本
- 支持 C++17 的 GCC/Clang
- OpenCV，包含 `core`、`imgcodecs`、`imgproc`、`highgui`、`calib3d`
- Eigen3

## 3. 安装依赖

Ubuntu/Debian 系统可执行：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev
```

确认工具可用：

```bash
cmake --version
g++ --version
pkg-config --modversion opencv4
```

如果系统安装的是自定义 OpenCV，需要在配置 CMake 时指定：

```bash
cmake -S . -B build -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4
```

## 4. 编译软件

在项目根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
```

生成的程序为：

```text
build/camcalib_app
```

重新编译：

```bash
cmake --build build -j
```

## 5. 数据目录

### 5.1 相机标定数据

相机图片放在同一目录中，例如：

```text
data/camera-1/
├── image_000.bmp
├── image_001.bmp
├── image_002.bmp
└── image_003.bmp
```

要求：

- 所有图片尺寸一致。
- 每张图片对应一个不同标定板位姿。
- 图片顺序必须与投影仪的 `pose_xxx` 顺序一致。
- 标定板需要完整出现在图像中。
- 圆点应具有足够对比度，避免过曝、欠曝和运动模糊。

### 5.2 投影仪标定数据

每个位姿包含 X、Y 两个方向，每个方向包含三频四步共 12 帧：

```text
data/projector/
├── pose_000/
│   ├── X/
│   │   ├── image_00.bmp
│   │   ├── ...
│   │   └── image_11.bmp
│   └── Y/
│       ├── image_00.bmp
│       ├── ...
│       └── image_11.bmp
├── pose_001/
│   ├── X/
│   └── Y/
└── pose_002/
    ├── X/
    └── Y/
```

每个方向的 12 帧必须按以下顺序排列：

```text
最高频率的4幅相移图
→ 中间频率的4幅相移图
→ 最低频率的4幅相移图
```

每组四步相移顺序必须与相位公式一致：

```text
I1, I2, I3, I4
```

建议：

- 位姿目录使用补零编号，例如 `pose_000`。
- X/Y 文件名应保证自然排序后与实际采集顺序一致。
- 每个位姿必须恰好包含 X 方向12帧和 Y 方向12帧。
- 所有相位图尺寸必须与相机图像尺寸一致。
- `pose_000` 必须对应相机数据中的第一张图片，以此类推。

## 6. 配置文件

主配置文件：

```text
config/calib_config.yaml
```

### 6.1 相机数据配置

```yaml
dataset:
  image_directory: "data/camera-1"
  image_extensions: [ ".jpg", ".jpeg", ".png", ".bmp" ]
  read_grayscale: 1
```

| 字段 | 说明 |
| --- | --- |
| `image_directory` | 相机标定图片目录 |
| `image_extensions` | 支持的图片扩展名 |
| `read_grayscale` | `1` 表示以灰度图读取 |

### 6.2 标定板配置

```yaml
board:
  rows: 9
  cols: 11
  spacing_mm: 15.0
```

| 字段 | 说明 |
| --- | --- |
| `rows` | 圆点行数 |
| `cols` | 圆点列数 |
| `spacing_mm` | 相邻圆心实际距离，单位毫米 |

`rows × cols` 必须等于每幅图像中的有效圆点数量。

### 6.3 圆点检测配置

```yaml
detector:
  min_contour_points: 20
  min_contour_area: 200.0
  max_contour_area: 100000.0
  max_axis_ratio: 1.5
  marker_count: 5
  marker_spacing: 150.0
  row_tolerance: 7.5
  enable_subpixel: 1
  black_circles_on_white_background: 0
```

| 字段 | 说明 |
| --- | --- |
| `min_contour_points` | 轮廓最少点数 |
| `min_contour_area` | 最小轮廓面积，单位为像素平方 |
| `max_contour_area` | 最大轮廓面积，单位为像素平方 |
| `max_axis_ratio` | 外接矩形最大长短轴比例 |
| `marker_count` | 定位 Marker 数量 |
| `marker_spacing` | Marker 设计间距 |
| `row_tolerance` | 圆点行排序容差 |
| `enable_subpixel` | 是否启用亚像素边缘细化 |
| `black_circles_on_white_background` | `1` 为白底黑圆，`0` 为黑底白圆 |

### 6.4 投影仪配置

```yaml
projector:
  enabled: 1
  method: "pseudo_camera"
  calibration_data_directory: "data/projector"
  phase_frequencies: [ 70.0, 64.0, 59.0 ]
  width: 1140
  height: 912
  min_valid_views: 3
```

| 字段 | 说明 |
| --- | --- |
| `enabled` | `1` 执行投影仪标定，`0` 跳过 |
| `method` | 当前使用 `pseudo_camera` |
| `calibration_data_directory` | 投影仪24帧位姿数据根目录 |
| `phase_frequencies` | 三个条纹频率，必须严格从高到低 |
| `width` | 投影仪有效分辨率宽度 |
| `height` | 投影仪有效分辨率高度 |
| `min_valid_views` | 投影仪标定最少有效位姿数，不小于3 |

相位到投影仪坐标的转换为：

```text
projectorX = phaseX × width  / (2π × highestFrequency)
projectorY = phaseY × height / (2π × highestFrequency)
```

### 6.5 日志配置

```yaml
logging:
  enabled: 1
  output_file: "debug_output/run.log"
```

日志包含数据读取、检测点数、相位解算状态、标定 RMS 和逐位姿重投影误差。

### 6.6 调试配置

```yaml
debug:
  enabled: 0
  save_images: 1
  show_windows: 0
  output_directory: "debug_output"
```

| 字段 | 说明 |
| --- | --- |
| `enabled` | 启用交互调试功能 |
| `save_images` | 保存检测和相位调试图 |
| `show_windows` | 使用 OpenCV 窗口显示结果 |
| `output_directory` | 调试输出根目录 |

无图形界面的服务器环境应设置：

```yaml
show_windows: 0
```

## 7. 运行软件

必须从项目根目录运行，因为程序使用相对配置路径：

```bash
./build/camcalib_app
```

正常输出示例：

```text
Camera calibration finished. RMS = ...
Projector calibration started.
Loaded projector calibration poses: ...
Absolute phase solved for pose_000
pose_000 projector points=99
Projector calibration finished. RMS = ...
```

## 8. 输出结果

### 8.1 标定参数

标定结果保存在配置目录：

```text
config/camera_calibration.yaml
config/projector_calibration.yaml
config/camera_projector_calibration.yaml
```

其中联合标定文件保存相机坐标系到投影仪坐标系的旋转矩阵、平移向量、本质矩阵和基础矩阵。

### 8.2 日志

默认保存到：

```text
debug_output/run.log
```

### 8.3 相机检测调试结果

每张相机图像对应一个目录：

```text
debug_output/image_000/
├── 00_pixel_edges.txt
├── 01_subpixel_edges.txt
├── 02_detected_edges.png
├── 03_fitted_centers.png
├── 04_sorted_markers.png
└── 05_sorted_board.png
```

### 8.4 投影仪绝对相位调试图

```text
debug_output/projector/
├── pose_000/
│   ├── x_absolute_phase.png
│   └── y_absolute_phase.png
└── pose_001/
    ├── x_absolute_phase.png
    └── y_absolute_phase.png
```

这些 PNG 是归一化彩色可视化结果，用于检查相位是否连续，不代表原始浮点相位数值。

## 9. 标定原理概述

### 9.1 相机标定

```text
标定板世界坐标
↔ 相机检测圆心像素
→ cv::calibrateCamera
```

输出相机内参、畸变系数以及每个位姿的旋转和平移。

### 9.2 投影仪伪相机标定

投影仪被视为一台反向相机：

```text
每个位姿 X/Y 各12帧
→ 三频四步绝对相位
→ 在相机圆心位置双线性采样相位
→ 转换为投影仪像素坐标
→ 标定板世界坐标与投影仪像素对应
→ cv::calibrateCamera
```

相机圆心和世界坐标直接复用相机检测结果，避免重复检测和点序不一致。

## 10. 常见问题

### 10.1 配置文件解析失败

OpenCV YAML 数组必须使用逗号：

```yaml
phase_frequencies: [ 70.0, 64.0, 59.0 ]
```

错误写法：

```yaml
phase_frequencies: [ 70.0 64.0 59.0 ]
```

### 10.2 没有检测到圆点

检查：

- `black_circles_on_white_background` 是否与标定板极性一致。
- `min_contour_area` 是否过大。
- `max_contour_area` 是否过小。
- `max_axis_ratio` 是否过于严格。
- 图像是否模糊、过曝或欠曝。

### 10.3 投影仪位姿被跳过

每个位姿必须满足：

- 存在 `X` 和 `Y` 目录。
- X/Y 各有且仅有12张有效图片。
- 所有图片尺寸一致。
- 图片可以被 OpenCV 正常读取。

### 10.4 相机位姿与投影仪位姿不匹配

相机图片数量必须与 `pose_xxx` 数量一致，并且顺序一一对应：

```text
相机第0张图片 ↔ pose_000
相机第1张图片 ↔ pose_001
```

### 10.5 绝对相位图不连续

检查：

- 三个频率是否与投射图案一致。
- 频率是否按照从高到低填写。
- 每组四步相移图片顺序是否正确。
- X/Y 图片是否放反。
- 场景中是否存在阴影、饱和区域或强反光。

### 10.6 投影仪坐标数量不足

投影仪坐标通过相机圆心位置的双线性相位采样得到。圆心落在图像边界、相位无效或转换后超出投影仪范围时，该点会被标记为无效并从标定观测中排除。

## 11. 开发目录说明

```text
app/            程序入口
config/         YAML 配置
include/        公共头文件
src/dataset/    相机和投影仪数据加载
src/imageProcess/ 图像处理与相位解算
src/detection/  圆点检测
src/projector/  相位匹配和投影仪坐标计算
src/calibration/ 标定算法
src/evaluation/ 重投影评价
src/pipeline/   相机与投影仪流程编排
src/utils/      配置、日志和结果输出
tests/          测试代码
```

## 12. 当前注意事项

- 当前投影仪方法仅支持 `pseudo_camera`。
- 相机图像与投影仪位姿依赖顺序对应，尚未使用清单文件匹配。
- 投影仪当前输出的是每个位姿下标定板到投影仪的外参。
- 若需要固定的相机到投影仪外参，还需要增加双目标定步骤。
- `CustomCalibrator::estimateIntrinsics()` 当前存在未完成实现，构建时可能出现无返回值警告；主流程使用 `OpenCvCalibrator`，不受该警告影响。
