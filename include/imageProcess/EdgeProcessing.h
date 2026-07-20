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

std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges_Canny(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
);

bool computeGradientField(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag);

// 双线性插值
bool bilinearSample(
    const cv::Mat& image,
    double x,
    double y,
    double& value);

// 单个像素点的亚像素细化
bool refineOneEdgePoint(
    const cv::Point& pixelPoint,
    const cv::Mat& gx,
    const cv::Mat& gy,
    const cv::Mat& mag,
    cv::Point2d& subpixelPoint,
    int searchRadius = 2,
    double minGradient = 1e-4);

// 单个轮廓
std::vector<cv::Point2d> refineOneContour(
    const std::vector<cv::Point>& pixelContour,
    const cv::Mat& gx,
    const cv::Mat& gy,
    const cv::Mat& mag);

void showCannyEdges(
    const cv::Mat& image,
    const std::string& windowName = "Canny Edges"
);

}  // namespace camcalib::image
