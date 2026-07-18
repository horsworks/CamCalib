#pragma once

#include "core/CalibrationData.h"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace camcalib {

struct BoardConfig {
    int rows = 0;
    int cols = 0;
    double spacingMm = 0.0;
};

struct DetectorConfig {
    int minContourPoints = 20;
    double maxAxisRatio = 1.5;
    int markerCount = 5;
    double markerSpacing = 150.0;
    double rowTolerance = 7.5;
    bool enableSubpixel = true;
};

struct DebugConfig {
    bool saveImages = true;
    bool showWindows = false;
    std::string outputDirectory = "debug_output";
};

struct CalibrationPipelineConfig {
    std::string imageDirectory;
    std::vector<std::string> imageExtensions;
    bool readGrayscale = true;

    BoardConfig board;
    DetectorConfig detector;

    bool logEnabled = true;
    std::string logOutputFile = "debug_output/run.log";
    bool debugMode = false;
    DebugConfig debug;
};

struct DatasetImage {
    std::string path;
    cv::Mat image;
};

struct CalibrationDataset {
    std::vector<DatasetImage> images;
    cv::Size imageSize;

    bool empty() const { return images.empty(); }
    size_t size() const { return images.size(); }
};

struct ViewObservation {
    std::string imagePath;
    cv::Size imageSize;
    std::vector<cv::Point2d> imagePoints;
    std::vector<cv::Point3f> objectPoints;

    bool valid = false;
    std::string failureReason;

    double detectionScore = 0.0;
    double reprojectionRmse = 0.0;
};

struct DetectionResult {
    std::vector<ViewObservation> views;
    std::vector<std::vector<std::vector<cv::Point>>> pixelEdges;     // 所有图像的边缘
    std::vector<std::vector<std::vector<cv::Point2d>>> subPixelEdges;  // 亚像素
    std::vector<std::vector<Circle>> fittedCircles;    // 拟合所有候选圆
    std::vector<std::vector<Circle>> sortedMarkerCircles;    // 定位使用的圆
    std::vector<Eigen::Matrix3d> homographies;            // 单映性矩阵
    std::vector<std::vector<Circle>> sortedBoardCircles;
};

struct CalibrationResult {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;

    double globalRmse = 0.0;
    bool converged = false;
    std::string solverName;
};

struct ViewError {
    size_t viewIndex = 0;
    double rmse = 0.0;
    double maxError = 0.0;
    cv::Point2d meanError;
};

struct EvaluationReport {
    double globalRmse = 0.0;
    double meanViewRmse = 0.0;
    double maxViewRmse = 0.0;
    std::vector<ViewError> views;
    std::vector<size_t> suspectedOutliers;
};

}  // namespace camcalib
