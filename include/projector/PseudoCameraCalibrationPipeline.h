#pragma once

#include "core/CalibrationTypes.h"
#include "projector/ProjectorPointMatcher.h"

namespace camcalib {

class PseudoCameraCalibrationPipeline {
public:
    PseudoCameraCalibrationPipeline(
        BoardConfig boardConfig,
        ProjectorConfig projectorConfig
    );

    CalibrationResult runCalibrateProject(
        const CalibrationDataset& dataset,
        const DetectionResult& cameraDetection,
        const std::vector<ProjectorCoordinateMaps>& coordinateMaps,
        DetectionResult& projectorDetection,
        EvaluationReport& evaluation
    ) const;

private:
    BoardConfig boardConfig_;
    ProjectorConfig projectorConfig_;
};

}  // namespace camcalib
