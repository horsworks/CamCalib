#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"

#include <vector>

namespace camcalib {

/** @brief 固定相机和投影仪内参，估计二者之间的相对外参。 */
class CameraProjectorCalibrator {
public:
    /** @brief 执行相机-投影仪联合标定。
     *  @param cameraViews 相机圆心、世界坐标和图像尺寸。
     *  @param projectorPoses 与相机位姿对应的投影仪像素坐标。
     *  @param cameraCalibration 相机单目标定结果。
     *  @param projectorCalibration 投影仪单目标定结果。
     *  @param minValidViews 要求的最少有效位姿数。
     *  @return 相机坐标系到投影仪坐标系的 R、T、E、F 和 RMSE。
     */
    CameraProjectorCalibrationResult calibrate(
        const std::vector<ViewObservation>& cameraViews,
        const std::vector<ProjectorPoseData>& projectorPoses,
        const CalibrationResult& cameraCalibration,
        const CalibrationResult& projectorCalibration,
        int minValidViews
    ) const;
};

}  // namespace camcalib
