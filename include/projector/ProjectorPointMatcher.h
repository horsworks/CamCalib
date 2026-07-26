#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"

#include <vector>

namespace camcalib {

class ProjectorPointMatcher {
public:
    void match(
        const std::vector<ViewObservation>& cameraViews,
        std::vector<ProjectorPoseData>& poses,
        const ProjectorConfig& projectorConfig
    ) const;
};

}  // namespace camcalib
