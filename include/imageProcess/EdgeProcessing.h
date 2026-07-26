#pragma once

#include "core/CalibrationTypes.h"
#include "core/CalibrationData.h"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace camcalib::image {

/** @brief 使用 Otsu 二值化提取并筛选圆形轮廓。
 *  @param images 输入图像数组。
 *  @param detectorConfig 轮廓点数、面积、轴比和前景极性配置。
 *  @return 按图像组织的整像素轮廓。
 */
std::vector<std::vector<std::vector<cv::Point>>> detectEdges(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& detectorConfig
);

/** @brief 使用 Canny 梯度提取并筛选圆形轮廓。
 *  @param images 输入图像数组。
 *  @param detectorConfig 轮廓筛选配置。
 *  @return 按图像组织的整像素轮廓。
 */
std::vector<std::vector<std::vector<cv::Point>>> detectEdgesGradient(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& detectorConfig
);

/** @brief 将整像素轮廓沿梯度方向细化到亚像素坐标。
 *  @param images 与轮廓一一对应的输入图像。
 *  @param pixelEdges 每幅图像的整像素轮廓。
 *  @return 按图像组织的亚像素轮廓。
 */
std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);

/** @brief 仅将整像素轮廓坐标转换为双精度坐标。
 *  @param pixelEdges 每幅图像的整像素轮廓。
 *  @return 不进行位置细化的双精度轮廓。
 */
std::vector<std::vector<std::vector<cv::Point2d>>> toSubPixelContours(
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);

/** @brief 使用梯度场细化整像素轮廓。
 *  @param images 与轮廓一一对应的输入图像。
 *  @param pixelEdges 每幅图像的整像素轮廓。
 *  @return 按图像组织的亚像素轮廓。
 */
std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges_Canny(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);

/** @brief 计算图像 X/Y 梯度及梯度幅值。
 *  @param img 输入灰度或彩色图像。
 *  @param gx 输出 X 方向梯度。
 *  @param gy 输出 Y 方向梯度。
 *  @param mag 输出梯度幅值。
 *  @return 输入有效且计算成功时返回 true。
 */
bool computeGradientField(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag);

/** @brief 在单通道图像中进行双线性采样。
 *  @param image 输入图像。
 *  @param x 采样横坐标。
 *  @param y 采样纵坐标。
 *  @param value 输出采样值。
 *  @return 坐标位于有效采样区域时返回 true。
 */
bool bilinearSample(
    const cv::Mat& image,
    double x,
    double y,
    double& value);

/** @brief 沿局部梯度方向细化一个边缘点。
 *  @param pixelPoint 输入整像素边缘点。
 *  @param gx X 方向梯度图。
 *  @param gy Y 方向梯度图。
 *  @param mag 梯度幅值图。
 *  @param subpixelPoint 输出亚像素边缘点。
 *  @param searchRadius 梯度方向搜索半径。
 *  @param minGradient 最小有效梯度。
 *  @return 找到有效亚像素位置时返回 true。
 */
bool refineOneEdgePoint(
    const cv::Point& pixelPoint,
    const cv::Mat& gx,
    const cv::Mat& gy,
    const cv::Mat& mag,
    cv::Point2d& subpixelPoint,
    int searchRadius = 2,
    double minGradient = 1e-4);

/** @brief 细化单条整像素轮廓。
 *  @param pixelContour 输入整像素轮廓。
 *  @param gx X 方向梯度图。
 *  @param gy Y 方向梯度图。
 *  @param mag 梯度幅值图。
 *  @return 有效的亚像素轮廓点。
 */
std::vector<cv::Point2d> refineOneContour(
    const std::vector<cv::Point>& pixelContour,
    const cv::Mat& gx,
    const cv::Mat& gy,
    const cv::Mat& mag);

/** @brief 显示输入图像的 Canny 轮廓。
 *  @param image 输入图像。
 *  @param windowName OpenCV 窗口名称。
 */
void showCannyEdges(
    const cv::Mat& image,
    const std::string& windowName = "Canny Edges"
);

}  // namespace camcalib::image
