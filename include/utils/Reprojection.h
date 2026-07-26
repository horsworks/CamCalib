#pragma once

#include "core/CalibrationData.h"
#include <opencv2/core.hpp>
#include <vector>

namespace camcalib::utils {

/** @brief 从已排序圆点中提取单精度图像坐标。
 *  @param sortedBoardCircles 每个位姿的已排序圆。
 *  @return 按位姿组织的圆心坐标。
 */
std::vector<std::vector<cv::Point2f>> collectImagePoints(
    const std::vector<std::vector<Circle>>& sortedBoardCircles
);

/** @brief 计算每个位姿的重投影均方根误差。
 *  @param objectPoints 世界坐标。
 *  @param imagePoints 实测图像坐标。
 *  @param rotationVectors 每个位姿旋转向量。
 *  @param translationVectors 每个位姿平移向量。
 *  @param cameraMatrix 内参矩阵。
 *  @param distCoeffs 畸变系数。
 *  @return 每个位姿的重投影 RMSE。
 */
std::vector<double> calculatePerImageReprojectionErrors(
    const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const std::vector<cv::Mat>& rotationVectors,
    const std::vector<cv::Mat>& translationVectors,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs
);

}  // namespace camcalib::utils
