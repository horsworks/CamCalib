#pragma once

#include "core/CalibrationData.h"
#include <Eigen/Dense>
#include <vector>

namespace camcalib::solver {

/** @brief 从候选圆中选取并排序定位 Marker。
 *  @param unsortedCenters 每个位姿的候选圆。
 *  @param markerCount 每个位姿期望 Marker 数量。
 *  @return 按固定规则排序的 Marker。
 */
std::vector<std::vector<Circle>> sortMarkerCenters(
    const std::vector<std::vector<Circle>>& unsortedCenters,
    int markerCount = 5
);

/** @brief 根据已排序 Marker 估计每个位姿单应矩阵。
 *  @param sortedMarkerCenters 每个位姿的已排序 Marker。
 *  @param markerSpacing Marker 模型间距。
 *  @return 每个位姿的 3x3 单应矩阵。
 */
std::vector<Eigen::Matrix3d> findHomography(
    const std::vector<std::vector<Circle>>& sortedMarkerCenters,
    double markerSpacing = 150.0
);

/** @brief 使用单应矩阵将候选圆映射到标定板平面并排序。
 *  @param homographies 每个位姿的单应矩阵。
 *  @param unsortedCircleCenters 每个位姿的候选圆。
 *  @param rowTolerance 按行聚类容差。
 *  @return 按标定板行列顺序排列的圆。
 */
std::vector<std::vector<Circle>> sortBoardCirclesByHomography(
    const std::vector<Eigen::Matrix3d>& homographies,
    const std::vector<std::vector<Circle>>& unsortedCircleCenters,
    double rowTolerance = 7.5
);

}  // namespace camcalib::solver
