#pragma once

#include "core/CalibrationData.h"
#include <opencv2/core.hpp>
#include <vector>

namespace camcalib::solver {

template<typename PointT>
Circle fitCircleToEdges(const std::vector<PointT>& points);

std::vector<std::vector<cv::Point3f>> generateWorldCoordinates(
    int imageCount,
    int calibRows,
    int calibCols,
    double centerDistance
);

}  // namespace camcalib::solver
