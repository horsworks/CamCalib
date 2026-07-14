#pragma once

#include "core/CalibrationTypes.h"
#include "core/CalibrationData.h"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace camcalib::image {

std::vector<std::vector<std::vector<cv::Point>>> detectEdges(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& detectorConfig
);

std::vector<std::vector<std::vector<cv::Point>>> detectEdgesGradient(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& detectorConfig
);

std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);

std::vector<std::vector<std::vector<cv::Point2d>>> toSubPixelContours(
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);

std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges_ray(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);


void showCannyEdges(
    const cv::Mat& image,
    const std::string& windowName = "Canny Edges"
);

}  // namespace camcalib::image
