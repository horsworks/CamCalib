#pragma once

#include "core/CalibrationData.h"
#include <Eigen/Dense>
#include <vector>

namespace camcalib::solver {

std::vector<std::vector<Circle>> sortMarkerCenters(
    const std::vector<std::vector<Circle>>& unsortedCenters,
    int markerCount = 5
);

std::vector<Eigen::Matrix3d> findHomography(
    const std::vector<std::vector<Circle>>& sortedMarkerCenters,
    double markerSpacing = 150.0
);

std::vector<std::vector<Circle>> sortBoardCirclesByHomography(
    const std::vector<Eigen::Matrix3d>& homographies,
    const std::vector<std::vector<Circle>>& unsortedCircleCenters,
    double rowTolerance = 7.5
);

}  // namespace camcalib::solver
