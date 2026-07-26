# CamCalib

CamCalib 是一个基于 C++17、OpenCV 和 Eigen 的相机-投影仪标定项目，支持圆点标定板检测、三频四步绝对相位解算、伪相机法投影仪标定以及相机-投影仪联合标定。

## 功能

- 白底黑圆、黑底白圆标定板检测
- 轮廓点数、面积和长短轴比例筛选
- 圆心亚像素提取与标定板点排序
- 相机内参、畸变和各位姿外参标定
- 三频四步相移绝对相位解算
- 相机圆心到投影仪像素坐标匹配
- 伪相机法投影仪标定
- 固定内参的相机-投影仪联合标定
- 重投影误差评价和 YAML 结果输出
- 圆点检测与绝对相位调试图保存

## 标定流程

```text
相机图像
→ 圆点检测与排序
→ 相机标定
→ 投影仪24帧位姿数据加载
→ X/Y绝对相位解算
→ 相机圆心位置相位采样
→ 投影仪像素坐标计算
→ 投影仪标定
→ 相机-投影仪联合标定
```

## 依赖

- CMake 3.16+
- C++17 编译器
- OpenCV：`core`、`imgcodecs`、`imgproc`、`highgui`、`calib3d`
- Eigen3

Ubuntu/Debian：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev
```

## 编译

```bash
git clone https://github.com/huanggeng2021/CamCalib.git
cd CamCalib

cmake -S . -B build
cmake --build build -j
```

生成程序：

```text
build/camcalib_app
```

## 数据组织

### 相机标定图片

```text
data/camera-1/
├── image_000.bmp
├── image_001.bmp
└── ...
```

### 投影仪标定图片

每个位姿包含 X、Y 两个方向，每个方向包含三频四步共12帧：

```text
data/projector/
├── pose_000/
│   ├── X/
│   │   ├── image_00.bmp
│   │   └── ... image_11.bmp
│   └── Y/
│       ├── image_00.bmp
│       └── ... image_11.bmp
├── pose_001/
└── ...
```

每个方向的图片顺序必须为：

```text
最高频率4帧 → 中间频率4帧 → 最低频率4帧
```

相机图片数量、顺序必须与 `pose_xxx` 一一对应。

## 配置

编辑：

```text
config/calib_config.yaml
```

核心配置示例：

```yaml
dataset:
  image_directory: "data/camera-1"
  image_extensions: [ ".jpg", ".jpeg", ".png", ".bmp" ]
  read_grayscale: 1

board:
  rows: 9
  cols: 11
  spacing_mm: 15.0

detector:
  min_contour_points: 20
  min_contour_area: 200.0
  max_contour_area: 100000.0
  max_axis_ratio: 1.5
  enable_subpixel: 1
  black_circles_on_white_background: 0

projector:
  enabled: 1
  method: "pseudo_camera"
  calibration_data_directory: "data/projector"
  phase_frequencies: [ 70.0, 64.0, 59.0 ]
  width: 1140
  height: 912
  min_valid_views: 3
```

`phase_frequencies` 必须与投射图案一致，并严格按照从高到低填写。

## 运行

在项目根目录执行：

```bash
./build/camcalib_app
```

程序依次执行：

1. 相机标定
2. 投影仪标定
3. 相机-投影仪联合标定

任一前置阶段失败时，后续阶段不会继续执行。

## 输出

标定结果：

```text
config/camera_calibration.yaml
config/projector_calibration.yaml
config/camera_projector_calibration.yaml
```

联合标定结果中的坐标变换定义为：

```text
X_projector = R × X_camera + T
```

调试输出：

```text
debug_output/
├── run.log
├── image_000/
│   ├── 02_detected_edges.png
│   ├── 04_sorted_markers.png
│   └── 05_sorted_board.png
└── projector/
    └── pose_000/
        ├── x_absolute_phase.png
        └── y_absolute_phase.png
```

## 文档

完整安装、配置、数据格式和排错说明见：

[PROJECT_PIPELINE.md](PROJECT_PIPELINE.md)

## 当前限制

- 投影仪当前仅实现伪相机法。
- 相机图片与投影仪位姿目前依靠顺序对应。
- 三频四步输入要求每个方向恰好12帧。
- `CustomCalibrator` 仍属于实验性实现，主流程使用 `OpenCvCalibrator`。

## License

本项目采用 [Apache License 2.0](LICENSE)。
