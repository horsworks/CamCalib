#include "evaluation/ReprojectionEvaluator.h"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

namespace camcalib {

EvaluationReport ReprojectionEvaluator::evaluate(
    const DetectionResult& detection,
    CalibrationResult& calibration
) const {
    EvaluationReport report;
    report.globalRmse = calibration.globalRmse;

    if(!calibration.converged){
        return report;
    }

    size_t calibrationViewIndex = 0;
    double rmseSum = 0.0;

    for(size_t viewIndex = 0; viewIndex < detection.views.size(); ++viewIndex){
        const ViewObservation& view = detection.views[viewIndex];
        if(!view.valid){
            continue;
        }
        if(calibrationViewIndex >= calibration.rotationVectors.size() ||
           calibrationViewIndex >= calibration.translationVectors.size()){
            break;
        }

        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(
            view.objectPoints,
            calibration.rotationVectors[calibrationViewIndex],
            calibration.translationVectors[calibrationViewIndex],
            calibration.cameraMatrix,
            calibration.distCoeffs,
            projectedPoints
        );

        double squaredErrorSum = 0.0;
        double maxError = 0.0;
        cv::Point2d meanError(0.0, 0.0);
        for(size_t pointIndex = 0; pointIndex < view.imagePoints.size() && pointIndex < projectedPoints.size();
            ++pointIndex){
            const cv::Point2d diff = view.imagePoints[pointIndex] - cv::Point2d(projectedPoints[pointIndex]);
            const double pointError = std::sqrt(diff.dot(diff));
            squaredErrorSum += diff.dot(diff);
            maxError = std::max(maxError, pointError);
            meanError += diff;
        }

        if(!view.imagePoints.empty()){
            meanError *= (1.0 / static_cast<double>(view.imagePoints.size()));
        }

        ViewError error;
        error.viewIndex = viewIndex;
        error.rmse = view.imagePoints.empty()
            ? 0.0
            : std::sqrt(squaredErrorSum / static_cast<double>(view.imagePoints.size()));
        error.maxError = maxError;
        error.meanError = meanError;

        rmseSum += error.rmse;
        report.maxViewRmse = std::max(report.maxViewRmse, error.rmse);
        if(report.globalRmse > 0.0 && error.rmse > report.globalRmse * 1.5){
            report.suspectedOutliers.push_back(viewIndex);
        }

        report.views.push_back(error);
        ++calibrationViewIndex;
    }

    if(!report.views.empty()){
        report.meanViewRmse = rmseSum / static_cast<double>(report.views.size());
    }

    return report;
}

}  // namespace camcalib
