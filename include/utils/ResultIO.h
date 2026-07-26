#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"
#include <Eigen/Dense>
#include <filesystem>
#include <opencv2/core.hpp>
#include <string>

namespace camcalib::utils {

bool prepareDebugOutputDirectory(const std::filesystem::path& debugRoot);

bool saveDetectionDebugResults(
    const std::filesystem::path& debugRoot,
    const CalibrationDataset& dataset,
    const DetectionResult& detection
);

bool saveProjectorPhaseDebugResults(
    const std::filesystem::path& debugRoot,
    const std::vector<ProjectorPoseData>& poses
);

void showDetectionDebugResults(
    const CalibrationDataset& dataset,
    const DetectionResult& detection
);

bool saveDebugImage(const std::filesystem::path& outputPath, const cv::Mat& image);

template<typename PointT>
bool saveEdgesToText(
    const std::filesystem::path& outputPath,
    const std::vector<std::vector<PointT>>& edges
);

void drawCenterPoints(cv::Mat& image, const std::vector<Circle>& centerPoints);

template<typename PointT>
cv::Mat renderEdgeAndCircleCenters(
    const cv::Mat& image,
    const std::vector<std::vector<PointT>>& edges,
    const std::vector<Circle>& centerPoints
);

cv::Mat renderSortedCircleCenters(
    const cv::Mat& image,
    const std::vector<Circle>& sortedCenterPoints
);

void showSortedCircleCenters(
    const cv::Mat& image,
    const std::vector<Circle>& sortedCenterPoints,
    const std::string& windowName
);

void showWarpedImage(
    const cv::Mat& image,
    const Eigen::Matrix3d& homography,
    const std::string& windowName
);

}  // namespace camcalib::utils
