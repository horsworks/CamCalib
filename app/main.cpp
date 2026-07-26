#include "pipeline/CalibrationPipeline.h"

#include <string>
#include <vector>

int main() {
    const std::string configPath = "config/calib_config.yaml";

    camcalib::CalibrationPipeline pipeline;
    camcalib::DetectionResult cameraDetection;
    camcalib::CalibrationResult cameraCalibration;
    // 相机标定
    if(!pipeline.runCameraCalibration(
           configPath,
           cameraDetection,
           cameraCalibration)){
        return 1;
    }

    std::vector<camcalib::ProjectorPoseData> projectorPoses;
    camcalib::CalibrationResult projectorCalibration;
    // 投影仪标定
    if(!pipeline.runProjectorCalibration(
           configPath,
           cameraDetection.views,
           projectorPoses,
           projectorCalibration)){
        return 1;
    }

    // 相机-投影仪联合标定
    if(!pipeline.runCameraProjectorCalibration(
           configPath,
           cameraDetection.views,
           projectorPoses,
           cameraCalibration,
           projectorCalibration)){
        return 1;
    }

    return 0;
}
