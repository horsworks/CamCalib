#include "projector/ProjectorPointMatcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace camcalib {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool samplePhase(
    const cv::Mat& absolutePhase,
    const cv::Point2d& imagePoint,
    double& phase
){
    if(absolutePhase.empty() || absolutePhase.type() != CV_32FC1){
        return false;
    }

    const int x0 = static_cast<int>(std::floor(imagePoint.x));
    const int y0 = static_cast<int>(std::floor(imagePoint.y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    if(x0 < 0 || y0 < 0 ||
       x1 >= absolutePhase.cols ||
       y1 >= absolutePhase.rows){
        return false;
    }

    const double phase00 = absolutePhase.at<float>(y0, x0);
    const double phase10 = absolutePhase.at<float>(y0, x1);
    const double phase01 = absolutePhase.at<float>(y1, x0);
    const double phase11 = absolutePhase.at<float>(y1, x1);
    if(!std::isfinite(phase00) ||
       !std::isfinite(phase10) ||
       !std::isfinite(phase01) ||
       !std::isfinite(phase11)){
        return false;
    }

    const double dx = imagePoint.x - static_cast<double>(x0);
    const double dy = imagePoint.y - static_cast<double>(y0);
    const double top = phase00 * (1.0 - dx) + phase10 * dx;
    const double bottom = phase01 * (1.0 - dx) + phase11 * dx;
    phase = top * (1.0 - dy) + bottom * dy;
    return true;
}

bool phaseToProjectorPoint(
    const ProjectorPoseData& pose,
    const cv::Point2d& cameraPoint,
    const ProjectorConfig& config,
    cv::Point2d& projectorPoint
){
    double xPhase = 0.0;
    double yPhase = 0.0;
    if(!samplePhase(pose.xAbsolutePhase, cameraPoint, xPhase) ||
       !samplePhase(pose.yAbsolutePhase, cameraPoint, yPhase)){
        return false;
    }

    const double highestFrequency =
        static_cast<double>(config.phaseFrequencies[0]);
    projectorPoint.x =
        xPhase * static_cast<double>(config.width) /
        (kTwoPi * highestFrequency);
    projectorPoint.y =
        yPhase * static_cast<double>(config.height) /
        (kTwoPi * highestFrequency);

    return std::isfinite(projectorPoint.x) &&
           std::isfinite(projectorPoint.y) &&
           projectorPoint.x >= 0.0 &&
           projectorPoint.y >= 0.0 &&
           projectorPoint.x < static_cast<double>(config.width) &&
           projectorPoint.y < static_cast<double>(config.height);
}

}  // namespace

void ProjectorPointMatcher::match(
    const std::vector<ViewObservation>& cameraViews,
    std::vector<ProjectorPoseData>& poses,
    const ProjectorConfig& projectorConfig
) const {
    for(ProjectorPoseData& pose : poses){
        pose.projectorPoints.clear();
    }

    const size_t viewCount = std::min(cameraViews.size(), poses.size());
    for(size_t viewIndex = 0; viewIndex < viewCount; ++viewIndex){
        const ViewObservation& cameraView = cameraViews[viewIndex];
        ProjectorPoseData& pose = poses[viewIndex];

        if(!cameraView.valid){
            continue;
        }

        pose.projectorPoints.reserve(cameraView.imagePoints.size());
        for(const cv::Point2d& cameraPoint : cameraView.imagePoints){
            cv::Point2d projectorPoint;
            if(!phaseToProjectorPoint(
                   pose,
                   cameraPoint,
                   projectorConfig,
                   projectorPoint)){
                const double invalidCoordinate =
                    std::numeric_limits<double>::quiet_NaN();
                projectorPoint = cv::Point2d(
                    invalidCoordinate,
                    invalidCoordinate
                );
            }
            pose.projectorPoints.push_back(projectorPoint);
        }
    }
}

}  // namespace camcalib
