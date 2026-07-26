#pragma once

#include "core/CalibrationTypes.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace camcalib {

struct ProjectorPoseData {
    std::string poseName;
    std::vector<cv::Mat> xImages;
    std::vector<cv::Mat> yImages;
    cv::Mat xAbsolutePhase;
    cv::Mat yAbsolutePhase;
    std::vector<cv::Point2d> projectorPoints;
};

class ProjectorDatasetLoader {
public:
    ProjectorDatasetLoader(
        ProjectorConfig projectorConfig,
        std::vector<std::string> imageExtensions
    );

    std::vector<ProjectorPoseData> load() const;

private:
    ProjectorConfig projectorConfig_;
    std::vector<std::string> imageExtensions_;
};

}  // namespace camcalib
