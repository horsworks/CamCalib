#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace camcalib {

/** @brief 圆形特征的边缘、圆心和标定板坐标信息。 */
struct Circle {
    std::vector<cv::Point2d> edge_points;  ///< 用于拟合圆的边缘点。
    cv::Point2d center;                    ///< 图像中的圆心坐标，单位为像素。
    double radius = 0.0;                   ///< 拟合圆半径，单位为像素。
    double board_row = 0.0;                ///< 圆心在标定板坐标系中的行坐标。
    double board_col = 0.0;                ///< 圆心在标定板坐标系中的列坐标。
};

}  // namespace camcalib
