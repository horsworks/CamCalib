#include "pipeline/CalibrationPipeline.h"

#include <string>

int main() {
    const std::string configPath = "config/calib_config.yaml";

    camcalib::CalibrationPipeline pipeline;
    camcalib::DetectionResult cameraDetection;
    // 相机标定
    pipeline.runCameraCalibration(configPath, cameraDetection);
    // 投影仪标定
    pipeline.runProjectorCalibration(configPath, cameraDetection.views);

    return 0;
}
