#pragma once

#include "core/CalibrationData.h"
#include <opencv2/core.hpp>
#include <vector>

namespace camcalib::solver {

/** @brief 使用闭式最小二乘方法拟合圆。
 *  @tparam PointT 支持 x、y 成员的 OpenCV 点类型。
 *  @param points 圆边缘点。
 *  @return 拟合圆心、半径和原始边缘点。
 */
template<typename PointT>
Circle fitCircleToEdges(const std::vector<PointT>& points);

/** @brief 为多个位姿生成平面圆点标定板世界坐标。
 *  @param imageCount 位姿数量。
 *  @param calibRows 标定板行数。
 *  @param calibCols 标定板列数。
 *  @param centerDistance 相邻圆心距离。
 *  @return 每个位姿共用的一组平面三维坐标。
 */
std::vector<std::vector<cv::Point3f>> generateWorldCoordinates(
    int imageCount,
    int calibRows,
    int calibCols,
    double centerDistance
);

}  // namespace camcalib::solver
