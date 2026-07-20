// 该文件负责自己实现opencv中的calibration函数及相关功能


#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib{

class CustomCalibrator {

public:

    explicit CustomCalibrator(BoardConfig boardConfig);   // 构造函数

    CalibrationResult calibrate(const CalibrationDataset& dataset, 
                                const DetectionResult& detetion) const;
private:
    // 1. 根据各视角的单应矩阵计算内参初值
    cv::Mat initializeIntrinsics(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2d>& imagePoints
    ) const;

    // 2. 根据内参和单应矩阵计算每张图像的外参
    void estimateExtrinsics(
        const cv::Mat& cameraMatrix,
        const std::vector<cv::Mat>& homographies,
        std::vector<cv::Mat>& rotationVectors,
        std::vector<cv::Mat>& translationVectors
    ) const;

    // 3. 初始化畸变系数
    cv::Mat initializeDistortion(
        const std::vector<std::vector<cv::Point3f>>& objectPoints,
        const std::vector<std::vector<cv::Point2d>>& imagePoints,
        const cv::Mat& cameraMatrix,
        const std::vector<cv::Mat>& rotationVectors,
        const std::vector<cv::Mat>& translationVectors
    ) const;

    // 4. 联合优化内参、畸变和外参
    void bundleAdjustment(
        const std::vector<std::vector<cv::Point3f>>& objectPoints,
        const std::vector<std::vector<cv::Point2d>>& imagePoints,
        CalibrationResult& result
    ) const;

    // 5. 计算整体重投影误差
    double calculateRmse(
        const std::vector<std::vector<cv::Point3f>>& objectPoints,
        const std::vector<std::vector<cv::Point2d>>& imagePoints,
        const CalibrationResult& result
    ) const;

private:

    BoardConfig boardConfig_;
};


}