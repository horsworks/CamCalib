#include "detection/CircleGridDetector.h"

#include "imageProcess/EdgeProcessing.h"
#include "solver/ClosedFormSolver.h"
#include "solver/Homography.h"
#include "utils/ResultIO.h"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace camcalib {
namespace {

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

std::string buildImageDebugFolder(size_t imageIndex){
    std::ostringstream imageFolderName;
    imageFolderName << "image_" << std::setw(3) << std::setfill('0') << imageIndex;
    return imageFolderName.str();
}

}  // namespace

CircleGridDetector::CircleGridDetector(CalibrationPipelineConfig config)
    : config_(std::move(config)) {}

DetectionResult CircleGridDetector::detect(const CalibrationDataset& dataset) const {
    DetectionResult result;
    if(dataset.empty()){
        return result;
    }

    std::vector<cv::Mat> images;
    images.reserve(dataset.images.size());
    result.views.reserve(dataset.images.size());
    for(const DatasetImage& datasetImage : dataset.images){
        images.push_back(datasetImage.image);

        ViewObservation view;
        view.imagePath = datasetImage.path;
        view.imageSize = datasetImage.image.size();
        result.views.push_back(view);
    }

    const bool shouldSaveDebugImages = config_.debug.saveImages;
    const std::filesystem::path debugRoot = config_.debug.outputDirectory;

    const auto pixelEdges = image::detectEdges(images, config_.detector);
    const auto subPixelEdges = config_.detector.enableSubpixel
        ? image::detectSubPixelEdges(images, pixelEdges)
        : image::toSubPixelContours(pixelEdges);

    result.fittedCircles.reserve(subPixelEdges.size());
    for(size_t imageIndex = 0; imageIndex < subPixelEdges.size(); ++imageIndex){
        std::vector<Circle> fittedCircles;
        fittedCircles.reserve(subPixelEdges[imageIndex].size());
        for(const std::vector<cv::Point2d>& contour : subPixelEdges[imageIndex]){
            fittedCircles.push_back(solver::fitCircleToEdges(contour));
        }
        result.fittedCircles.push_back(fittedCircles);

        ViewObservation& view = result.views[imageIndex];
        view.detectionScore = fittedCircles.empty()
            ? 0.0
            : static_cast<double>(fittedCircles.size()) /
                  static_cast<double>(config_.board.rows * config_.board.cols);
        if(fittedCircles.empty()){
            view.failureReason = "No circular contours were fitted.";
        }

        if(shouldSaveDebugImages){
            const std::filesystem::path imageDebugDir = debugRoot / buildImageDebugFolder(imageIndex);
            utils::saveEdgesToText(imageDebugDir / "00_pixel_edges.txt", pixelEdges[imageIndex]);
            utils::saveEdgesToText(imageDebugDir / "01_subpixel_edges.txt", subPixelEdges[imageIndex]);
            utils::saveDebugImage(
                imageDebugDir / "02_detected_edges.png",
                utils::renderEdgeAndCircleCenters(images[imageIndex], subPixelEdges[imageIndex], fittedCircles)
            );
            utils::saveDebugImage(
                imageDebugDir / "03_fitted_centers.png",
                utils::renderSortedCircleCenters(images[imageIndex], fittedCircles)
            );
        }
    }

    result.sortedMarkerCircles = solver::sortMarkerCenters(result.fittedCircles, config_.detector.markerCount);
    result.homographies = solver::findHomography(result.sortedMarkerCircles, config_.detector.markerSpacing);
    result.sortedBoardCircles = solver::sortBoardCirclesByHomography(
        result.homographies,
        result.fittedCircles,
        config_.detector.rowTolerance
    );

    const std::vector<cv::Point3f> objectPoints = toObjectPoints(config_.board);
    const size_t expectedPointCount = static_cast<size_t>(config_.board.rows * config_.board.cols);

    for(size_t imageIndex = 0; imageIndex < result.views.size(); ++imageIndex){
        ViewObservation& view = result.views[imageIndex];

        if(imageIndex >= result.sortedMarkerCircles.size() || result.sortedMarkerCircles[imageIndex].empty()){
            view.failureReason = "Marker sorting failed.";
            continue;
        }

        if(imageIndex >= result.sortedBoardCircles.size() || result.sortedBoardCircles[imageIndex].empty()){
            view.failureReason = "Board sorting failed.";
            continue;
        }

        if(result.sortedBoardCircles[imageIndex].size() != expectedPointCount){
            view.failureReason = "Sorted board point count mismatch.";
            continue;
        }

        view.imagePoints = toImagePoints(result.sortedBoardCircles[imageIndex]);
        view.objectPoints = objectPoints;
        view.valid = true;
        view.failureReason.clear();

        if(shouldSaveDebugImages){
            const std::filesystem::path imageDebugDir = debugRoot / buildImageDebugFolder(imageIndex);
            utils::saveDebugImage(
                imageDebugDir / "04_sorted_markers.png",
                utils::renderSortedCircleCenters(images[imageIndex], result.sortedMarkerCircles[imageIndex])
            );
            utils::saveDebugImage(
                imageDebugDir / "05_sorted_board.png",
                utils::renderSortedCircleCenters(images[imageIndex], result.sortedBoardCircles[imageIndex])
            );
        }
    }

    if(config_.debugMode && config_.debug.showWindows){
        for(size_t imageIndex = 0; imageIndex < result.sortedMarkerCircles.size(); ++imageIndex){
            if(!result.sortedMarkerCircles[imageIndex].empty()){
                utils::showSortedCircleCenters(
                    images[imageIndex],
                    result.sortedMarkerCircles[imageIndex],
                    "Sorted Circle Centers " + std::to_string(imageIndex)
                );
            }
        }

        for(size_t imageIndex = 0; imageIndex < result.sortedBoardCircles.size() &&
                                   imageIndex < result.homographies.size(); ++imageIndex){
            if(result.sortedBoardCircles[imageIndex].empty()){
                continue;
            }

            utils::showWarpedImage(
                images[imageIndex],
                result.homographies[imageIndex],
                "Warped Image " + std::to_string(imageIndex)
            );
            utils::showSortedCircleCenters(
                images[imageIndex],
                result.sortedBoardCircles[imageIndex],
                "Sorted Board Circles " + std::to_string(imageIndex)
            );
        }
    }

    return result;
}

}  // namespace camcalib
