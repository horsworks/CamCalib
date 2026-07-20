#include "detection/CircleGridDetector.h"

#include "imageProcess/EdgeProcessing.h"
#include "solver/ClosedFormSolver.h"
#include "solver/Homography.h"

#include <utility>

namespace camcalib {
namespace {

using PixelContours = std::vector<std::vector<std::vector<cv::Point>>>;
using SubPixelContours = std::vector<std::vector<std::vector<cv::Point2d>>>;
using CirclesByView = std::vector<std::vector<Circle>>;

std::vector<cv::Point2d> toImagePoints(const std::vector<Circle>& circles){
    std::vector<cv::Point2d> imagePoints;
    imagePoints.reserve(circles.size());
    for(const Circle& circle : circles){
        imagePoints.push_back(circle.center);
    }
    return imagePoints;
}

std::vector<cv::Point3f> toObjectPoints(const BoardConfig& board){
    const auto worldPoints = solver::generateWorldCoordinates(1, board.rows, board.cols, board.spacingMm);
    return worldPoints.empty() ? std::vector<cv::Point3f>{} : worldPoints.front();
}

std::vector<cv::Mat> collectImages(const CalibrationDataset& dataset){
    std::vector<cv::Mat> images;
    images.reserve(dataset.images.size());
    for(const DatasetImage& datasetImage : dataset.images){
        images.push_back(datasetImage.image);
    }
    return images;
}

std::vector<ViewObservation> createViews(const CalibrationDataset& dataset){
    std::vector<ViewObservation> views;
    views.reserve(dataset.images.size());
    for(const DatasetImage& datasetImage : dataset.images){
        ViewObservation view;
        view.imagePath = datasetImage.path;
        view.imageSize = datasetImage.image.size();
        views.push_back(std::move(view));
    }
    return views;
}

PixelContours extractPixelContours(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& config
){
    return image::detectEdges(images, config);
}

SubPixelContours buildFittingContours(
    const std::vector<cv::Mat>& images,
    const PixelContours& pixelContours,
    bool enableSubpixel
){
    return enableSubpixel
        ? image::detectSubPixelEdges(images, pixelContours)
        : image::toSubPixelContours(pixelContours);
}

CirclesByView fitCircles(const SubPixelContours& contoursByView){
    CirclesByView circlesByView;
    circlesByView.reserve(contoursByView.size());
    for(const auto& contours : contoursByView){
        std::vector<Circle> fittedCircles;
        fittedCircles.reserve(contours.size());
        for(const std::vector<cv::Point2d>& contour : contours){
            fittedCircles.push_back(solver::fitCircleToEdges(contour));
        }
        circlesByView.push_back(std::move(fittedCircles));
    }
    return circlesByView;
}

void updateDetectionScores(
    const CirclesByView& circlesByView,
    const BoardConfig& board,
    std::vector<ViewObservation>& views
){
    const double expectedCircleCount = static_cast<double>(board.rows * board.cols);
    for(size_t viewIndex = 0; viewIndex < views.size(); ++viewIndex){
        ViewObservation& view = views[viewIndex];
        if(viewIndex >= circlesByView.size()){
            view.failureReason = "Circle fitting result is missing.";
            continue;
        }

        const std::vector<Circle>& circles = circlesByView[viewIndex];
        view.detectionScore = circles.empty() || expectedCircleCount <= 0.0
            ? 0.0
            : static_cast<double>(circles.size()) / expectedCircleCount;
        if(circles.empty()){
            view.failureReason = "No circular contours were fitted.";
        }
    }
}

void finalizeViews(
    const BoardConfig& board,
    const CirclesByView& sortedMarkerCircles,
    const CirclesByView& sortedBoardCircles,
    std::vector<ViewObservation>& views
){
    const std::vector<cv::Point3f> objectPoints = toObjectPoints(board);
    const size_t expectedPointCount = static_cast<size_t>(board.rows * board.cols);

    for(size_t viewIndex = 0; viewIndex < views.size(); ++viewIndex){
        ViewObservation& view = views[viewIndex];

        if(viewIndex >= sortedMarkerCircles.size() || sortedMarkerCircles[viewIndex].empty()){
            view.failureReason = "Marker sorting failed.";
            continue;
        }

        if(viewIndex >= sortedBoardCircles.size() || sortedBoardCircles[viewIndex].empty()){
            view.failureReason = "Board sorting failed.";
            continue;
        }

        if(sortedBoardCircles[viewIndex].size() != expectedPointCount){
            view.failureReason = "Sorted board point count mismatch.";
            continue;
        }

        view.imagePoints = toImagePoints(sortedBoardCircles[viewIndex]);
        view.objectPoints = objectPoints;
        view.valid = true;
        view.failureReason.clear();
    }
}

}  // namespace

CircleGridDetector::CircleGridDetector(BoardConfig boardConfig, DetectorConfig detectorConfig)
    : boardConfig_(std::move(boardConfig)), detectorConfig_(std::move(detectorConfig)) {}

DetectionResult CircleGridDetector::detect(const CalibrationDataset& dataset) const {
    DetectionResult result;
    if(dataset.empty()){
        return result;
    }

    const std::vector<cv::Mat> images = collectImages(dataset);
    result.views = createViews(dataset);    // 记录图像状态

    result.pixelEdges = extractPixelContours(images, detectorConfig_);   // 整像素边缘
    result.subPixelEdges = buildFittingContours(
        images,
        result.pixelEdges,
        detectorConfig_.enableSubpixel
    );
    result.fittedCircles = fitCircles(result.subPixelEdges);   //  以圆形来拟合圆心， 实际不为圆形

    // 添加亚像素提取

    updateDetectionScores(result.fittedCircles, boardConfig_, result.views);

    result.sortedMarkerCircles = solver::sortMarkerCenters(
        result.fittedCircles,
        detectorConfig_.markerCount
    );
    result.homographies = solver::findHomography(
        result.sortedMarkerCircles,
        detectorConfig_.markerSpacing
    );
    result.sortedBoardCircles = solver::sortBoardCirclesByHomography(
        result.homographies,
        result.fittedCircles,
        detectorConfig_.rowTolerance
    );

    finalizeViews(
        boardConfig_,
        result.sortedMarkerCircles,
        result.sortedBoardCircles,
        result.views
    );

    return result;
}

}  // namespace camcalib
