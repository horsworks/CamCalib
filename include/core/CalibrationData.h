#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace camcalib {

struct Circle {
    std::vector<cv::Point2d> edge_points;
    cv::Point2d center;
    double radius = 0.0;
    double board_row = 0.0;
    double board_col = 0.0;
};

}  // namespace camcalib
