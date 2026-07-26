#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"

#include <array>
#include <string>
#include <vector>

namespace camcalib {

class CalibrationPipeline {
public:
    bool runCameraCalibration(
        const std::string& configPath,
        DetectionResult& cameraDetection
    ) const;
    void runProjectorCalibration(
        const std::string& configPath,
        const std::vector<ViewObservation>& cameraViews
    ) const;

private:
    DetectionResult buildProjectorDetection(
        const std::vector<ViewObservation>& cameraViews,
        const std::vector<ProjectorPoseData>& poses,
        cv::Size projectorSize
    ) const;

    bool solveProjectorPhases(
        std::vector<ProjectorPoseData>& poses,
        const std::array<float, 3>& frequencies,
        int minValidViews
    ) const;

    CalibrationResult calibrateAndEvaluate(
        const CalibrationDataset& dataset,
        DetectionResult& detection,
        const BoardConfig& board,
        EvaluationReport& evaluation
    ) const;
};

}  // namespace camcalib
