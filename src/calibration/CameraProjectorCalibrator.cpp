#include "calibration/CameraProjectorCalibrator.h"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

namespace camcalib {

CameraProjectorCalibrationResult CameraProjectorCalibrator::calibrate(
    const std::vector<ViewObservation>& cameraViews,
    const std::vector<ProjectorPoseData>& projectorPoses,
    const CalibrationResult& cameraCalibration,
    const CalibrationResult& projectorCalibration,
    int minValidViews
) const {
    CameraProjectorCalibrationResult result;
    if(!cameraCalibration.converged ||
       !projectorCalibration.converged ||
       cameraCalibration.cameraMatrix.empty() ||
       projectorCalibration.cameraMatrix.empty()){
        return result;
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> cameraPoints;
    std::vector<std::vector<cv::Point2f>> projectorPoints;
    cv::Size cameraImageSize;

    const size_t viewCount = std::min(cameraViews.size(), projectorPoses.size());
    for(size_t viewIndex = 0; viewIndex < viewCount; ++viewIndex){
        const ViewObservation& cameraView = cameraViews[viewIndex];
        const ProjectorPoseData& projectorPose = projectorPoses[viewIndex];
        if(!cameraView.valid){
            continue;
        }

        std::vector<cv::Point3f> viewObjectPoints;
        std::vector<cv::Point2f> viewCameraPoints;
        std::vector<cv::Point2f> viewProjectorPoints;
        const size_t pointCount = std::min({
            cameraView.objectPoints.size(),
            cameraView.imagePoints.size(),
            projectorPose.projectorPoints.size()
        });

        for(size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex){
            const cv::Point2d& projectorPoint =
                projectorPose.projectorPoints[pointIndex];
            if(!std::isfinite(projectorPoint.x) ||
               !std::isfinite(projectorPoint.y)){
                continue;
            }

            viewObjectPoints.push_back(cameraView.objectPoints[pointIndex]);
            viewCameraPoints.emplace_back(cameraView.imagePoints[pointIndex]);
            viewProjectorPoints.emplace_back(projectorPoint);
        }

        if(viewObjectPoints.size() < 4){
            continue;
        }

        objectPoints.push_back(std::move(viewObjectPoints));
        cameraPoints.push_back(std::move(viewCameraPoints));
        projectorPoints.push_back(std::move(viewProjectorPoints));
        if(cameraImageSize.empty()){
            cameraImageSize = cameraView.imageSize;
        }
    }

    if(static_cast<int>(objectPoints.size()) < minValidViews ||
       cameraImageSize.empty()){
        return result;
    }

    cv::Mat cameraMatrix = cameraCalibration.cameraMatrix.clone();
    cv::Mat cameraDistCoeffs = cameraCalibration.distCoeffs.clone();
    cv::Mat projectorMatrix = projectorCalibration.cameraMatrix.clone();
    cv::Mat projectorDistCoeffs = projectorCalibration.distCoeffs.clone();

    result.globalRmse = cv::stereoCalibrate(
        objectPoints,
        cameraPoints,
        projectorPoints,
        cameraMatrix,
        cameraDistCoeffs,
        projectorMatrix,
        projectorDistCoeffs,
        cameraImageSize,
        result.rotation,
        result.translation,
        result.essentialMatrix,
        result.fundamentalMatrix,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(
            cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
            100,
            1e-8
        )
    );
    result.converged = !result.rotation.empty() && !result.translation.empty();
    return result;
}

}  // namespace camcalib
