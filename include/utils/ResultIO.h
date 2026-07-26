#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"
#include <Eigen/Dense>
#include <filesystem>
#include <opencv2/core.hpp>
#include <string>

namespace camcalib::utils {

/** @brief 创建调试输出根目录。
 *  @param debugRoot 调试输出目录。
 *  @return 目录存在或创建成功时返回 true。
 */
bool prepareDebugOutputDirectory(const std::filesystem::path& debugRoot);

/** @brief 保存相机圆点检测各阶段的文本和图像结果。 */
bool saveDetectionDebugResults(
    const std::filesystem::path& debugRoot,
    const CalibrationDataset& dataset,
    const DetectionResult& detection
);

/** @brief 保存每个位姿 X/Y 绝对相位的彩色可视化图。 */
bool saveProjectorPhaseDebugResults(
    const std::filesystem::path& debugRoot,
    const std::vector<ProjectorPoseData>& poses
);

/** @brief 将相机或投影仪标定结果保存为 OpenCV YAML 文件。
 *  @param outputPath YAML 输出路径。
 *  @param deviceName 设备名称，例如 camera 或 projector。
 *  @param calibration 待保存的标定结果。
 *  @return 文件成功写入时返回 true。
 */
bool saveCalibrationResult(
    const std::filesystem::path& outputPath,
    const std::string& deviceName,
    const CalibrationResult& calibration
);

/** @brief 将相机-投影仪联合标定结果保存为 OpenCV YAML 文件。
 *  @param outputPath YAML 输出路径。
 *  @param calibration 相机到投影仪的相对外参结果。
 *  @return 文件成功写入时返回 true。
 */
bool saveCameraProjectorCalibrationResult(
    const std::filesystem::path& outputPath,
    const CameraProjectorCalibrationResult& calibration
);

/** @brief 使用 OpenCV 窗口显示检测和排序结果。 */
void showDetectionDebugResults(
    const CalibrationDataset& dataset,
    const DetectionResult& detection
);

/** @brief 创建父目录并保存图像。
 *  @param outputPath 输出文件路径。
 *  @param image 待保存图像。
 *  @return 图像成功写入时返回 true。
 */
bool saveDebugImage(const std::filesystem::path& outputPath, const cv::Mat& image);

/** @brief 将多条边缘轮廓保存为文本。 */
template<typename PointT>
bool saveEdgesToText(
    const std::filesystem::path& outputPath,
    const std::vector<std::vector<PointT>>& edges
);

/** @brief 在图像上绘制拟合圆心。 */
void drawCenterPoints(cv::Mat& image, const std::vector<Circle>& centerPoints);

/** @brief 渲染边缘轮廓、采样点和拟合圆心。 */
template<typename PointT>
cv::Mat renderEdgeAndCircleCenters(
    const cv::Mat& image,
    const std::vector<std::vector<PointT>>& edges,
    const std::vector<Circle>& centerPoints
);

/** @brief 渲染已排序圆的圆周、圆心和序号。 */
cv::Mat renderSortedCircleCenters(
    const cv::Mat& image,
    const std::vector<Circle>& sortedCenterPoints
);

/** @brief 在 OpenCV 窗口显示已排序圆。 */
void showSortedCircleCenters(
    const cv::Mat& image,
    const std::vector<Circle>& sortedCenterPoints,
    const std::string& windowName
);

/** @brief 使用单应矩阵显示标定板平面矫正图像。 */
void showWarpedImage(
    const cv::Mat& image,
    const Eigen::Matrix3d& homography,
    const std::string& windowName
);

}  // namespace camcalib::utils
