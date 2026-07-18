#include "utils/ResultIO.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <type_traits>

namespace camcalib::utils {
namespace {

std::filesystem::path imageDebugDirectory(
    const std::filesystem::path& debugRoot,
    size_t imageIndex
){
    std::ostringstream folderName;
    folderName << "image_" << std::setw(3) << std::setfill('0') << imageIndex;
    return debugRoot / folderName.str();
}

}  // namespace

bool prepareDebugOutputDirectory(const std::filesystem::path& debugRoot){
    try{
        std::filesystem::create_directories(debugRoot);
    }catch(const std::filesystem::filesystem_error&){
        return false;
    }
    return true;
}

bool saveDetectionDebugResults(
    const std::filesystem::path& debugRoot,
    const CalibrationDataset& dataset,
    const DetectionResult& detection
){
    bool allSaved = true;
    for(size_t imageIndex = 0; imageIndex < dataset.images.size(); ++imageIndex){
        const std::filesystem::path outputDir = imageDebugDirectory(debugRoot, imageIndex);

        if(imageIndex < detection.pixelEdges.size()){
            allSaved = saveEdgesToText(
                outputDir / "00_pixel_edges.txt",
                detection.pixelEdges[imageIndex]
            ) && allSaved;
        }
        if(imageIndex < detection.subPixelEdges.size()){
            allSaved = saveEdgesToText(
                outputDir / "01_subpixel_edges.txt",
                detection.subPixelEdges[imageIndex]
            ) && allSaved;
        }
        if(imageIndex < detection.subPixelEdges.size() &&
           imageIndex < detection.fittedCircles.size()){
            allSaved = saveDebugImage(
                outputDir / "02_detected_edges.png",
                renderEdgeAndCircleCenters(
                    dataset.images[imageIndex].image,
                    detection.subPixelEdges[imageIndex],
                    detection.fittedCircles[imageIndex]
                )
            ) && allSaved;
            allSaved = saveDebugImage(
                outputDir / "03_fitted_centers.png",
                renderSortedCircleCenters(
                    dataset.images[imageIndex].image,
                    detection.fittedCircles[imageIndex]
                )
            ) && allSaved;
        }
        if(imageIndex < detection.sortedMarkerCircles.size()){
            allSaved = saveDebugImage(
                outputDir / "04_sorted_markers.png",
                renderSortedCircleCenters(
                    dataset.images[imageIndex].image,
                    detection.sortedMarkerCircles[imageIndex]
                )
            ) && allSaved;
        }
        if(imageIndex < detection.sortedBoardCircles.size()){
            allSaved = saveDebugImage(
                outputDir / "05_sorted_board.png",
                renderSortedCircleCenters(
                    dataset.images[imageIndex].image,
                    detection.sortedBoardCircles[imageIndex]
                )
            ) && allSaved;
        }
    }
    return allSaved;
}

void showDetectionDebugResults(
    const CalibrationDataset& dataset,
    const DetectionResult& detection
){
    const size_t imageCount = dataset.images.size();
    for(size_t imageIndex = 0;
        imageIndex < imageCount && imageIndex < detection.sortedMarkerCircles.size();
        ++imageIndex){
        if(!detection.sortedMarkerCircles[imageIndex].empty()){
            showSortedCircleCenters(
                dataset.images[imageIndex].image,
                detection.sortedMarkerCircles[imageIndex],
                "Sorted Circle Centers " + std::to_string(imageIndex)
            );
        }
    }

    for(size_t imageIndex = 0;
        imageIndex < imageCount &&
        imageIndex < detection.sortedBoardCircles.size() &&
        imageIndex < detection.homographies.size();
        ++imageIndex){
        if(detection.sortedBoardCircles[imageIndex].empty()){
            continue;
        }
        showWarpedImage(
            dataset.images[imageIndex].image,
            detection.homographies[imageIndex],
            "Warped Image " + std::to_string(imageIndex)
        );
        showSortedCircleCenters(
            dataset.images[imageIndex].image,
            detection.sortedBoardCircles[imageIndex],
            "Sorted Board Circles " + std::to_string(imageIndex)
        );
    }
}

bool saveDebugImage(const std::filesystem::path& outputPath, const cv::Mat& image){
    try{
        std::filesystem::create_directories(outputPath.parent_path());
    }catch(const std::filesystem::filesystem_error&){
        return false;
    }

    return cv::imwrite(outputPath.string(), image);
}

template<typename PointT>
bool saveEdgesToText(
    const std::filesystem::path& outputPath,
    const std::vector<std::vector<PointT>>& edges
){
    try{
        std::filesystem::create_directories(outputPath.parent_path());
    }catch(const std::filesystem::filesystem_error&){
        return false;
    }

    std::ofstream outFile(outputPath);
    if(!outFile.is_open()){
        return false;
    }

    outFile << std::fixed << std::setprecision(6);
    outFile << "# contour_index point_index x y\n";
    for(size_t contourIndex = 0; contourIndex < edges.size(); ++contourIndex){
        for(size_t pointIndex = 0; pointIndex < edges[contourIndex].size(); ++pointIndex){
            outFile
                << contourIndex << ' '
                << pointIndex << ' '
                << static_cast<double>(edges[contourIndex][pointIndex].x) << ' '
                << static_cast<double>(edges[contourIndex][pointIndex].y) << '\n';
        }
    }

    return true;
}

void drawCenterPoints(cv::Mat& image, const std::vector<Circle>& centerPoints){
    for(const Circle& circle : centerPoints){
        if(circle.radius <= 0.0){
            continue;
        }

        const cv::Point center(
            static_cast<int>(std::round(circle.center.x)),
            static_cast<int>(std::round(circle.center.y))
        );
        cv::circle(image, center, 4, cv::Scalar(255, 0, 0), -1);
        cv::drawMarker(image, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 16, 2);
    }
}

template<typename PointT>
cv::Mat renderEdgeAndCircleCenters(
    const cv::Mat& image,
    const std::vector<std::vector<PointT>>& edges,
    const std::vector<Circle>& centerPoints
){
    cv::Mat display;
    if(image.channels() == 1){
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    }else{
        display = image.clone();
    }

    cv::Mat overlay = display.clone();
    constexpr bool isSubPixel = std::is_floating_point_v<typename PointT::value_type>;
    constexpr int pointShift = isSubPixel ? 8 : 0;
    constexpr int pointScale = 1 << pointShift;

    for(size_t contourIndex = 0; contourIndex < edges.size(); ++contourIndex){
        if(edges[contourIndex].empty()){
            continue;
        }

        cv::Point2d meanPoint(0.0, 0.0);
        std::vector<cv::Point> polyline;
        polyline.reserve(edges[contourIndex].size());

        for(const PointT& point : edges[contourIndex]){
            meanPoint += cv::Point2d(static_cast<double>(point.x), static_cast<double>(point.y));
            polyline.emplace_back(
                static_cast<int>(std::round(static_cast<double>(point.x) * pointScale)),
                static_cast<int>(std::round(static_cast<double>(point.y) * pointScale))
            );
        }

        for(size_t pointIndex = 1; pointIndex < polyline.size(); ++pointIndex){
            cv::line(
                overlay,
                polyline[pointIndex - 1],
                polyline[pointIndex],
                cv::Scalar(0, 0, 255),
                1,
                cv::LINE_AA,
                pointShift
            );
        }

        if(polyline.size() > 2){
            cv::line(
                overlay,
                polyline.back(),
                polyline.front(),
                cv::Scalar(0, 0, 255),
                1,
                cv::LINE_AA,
                pointShift
            );
        }

        const size_t markerStep = std::max<size_t>(1, edges[contourIndex].size() / 24);
        for(size_t pointIndex = 0; pointIndex < edges[contourIndex].size(); pointIndex += markerStep){
            const PointT& point = edges[contourIndex][pointIndex];
            cv::circle(
                overlay,
                cv::Point(
                    static_cast<int>(std::round(static_cast<double>(point.x))),
                    static_cast<int>(std::round(static_cast<double>(point.y)))
                ),
                1,
                cv::Scalar(80, 220, 255),
                -1,
                cv::LINE_AA
            );
        }

        meanPoint *= (1.0 / static_cast<double>(edges[contourIndex].size()));
        cv::putText(
            overlay,
            std::to_string(contourIndex),
            cv::Point(
                static_cast<int>(std::round(meanPoint.x)),
                std::max(15, static_cast<int>(std::round(meanPoint.y)) - 5)
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA
        );
    }

    cv::addWeighted(overlay, 0.75, display, 0.25, 0.0, display);
    drawCenterPoints(display, centerPoints);
    return display;
}

cv::Mat renderSortedCircleCenters(const cv::Mat& image, const std::vector<Circle>& sortedCenterPoints){
    cv::Mat display;
    if(image.channels() == 1){
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    }else{
        display = image.clone();
    }

    for(size_t circleIndex = 0; circleIndex < sortedCenterPoints.size(); ++circleIndex){
        const Circle& circle = sortedCenterPoints[circleIndex];
        if(circle.radius <= 0.0){
            continue;
        }

        const cv::Point center(
            static_cast<int>(std::round(circle.center.x)),
            static_cast<int>(std::round(circle.center.y))
        );

        cv::circle(display, center, static_cast<int>(std::round(circle.radius)), cv::Scalar(0, 255, 255), 2);
        cv::circle(display, center, 4, cv::Scalar(0, 0, 255), -1);
        cv::drawMarker(display, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 16, 2);

        cv::Point textPosition(center.x + 8, center.y - 8);
        textPosition.x = std::max(0, std::min(textPosition.x, display.cols - 40));
        textPosition.y = std::max(20, std::min(textPosition.y, display.rows - 5));

        cv::putText(
            display,
            std::to_string(circleIndex),
            textPosition,
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA
        );
    }

    return display;
}

void showSortedCircleCenters(
    const cv::Mat& image,
    const std::vector<Circle>& sortedCenterPoints,
    const std::string& windowName
){
    cv::Mat display = renderSortedCircleCenters(image, sortedCenterPoints);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1200, 1000);
    cv::imshow(windowName, display);
    cv::waitKey(0);
}

void showWarpedImage(
    const cv::Mat& image,
    const Eigen::Matrix3d& homography,
    const std::string& windowName
){
    cv::Mat display;
    if(image.channels() == 1){
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    }else{
        display = image.clone();
    }

    cv::Mat homographyCv(3, 3, CV_64F);
    for(int row = 0; row < 3; ++row){
        for(int col = 0; col < 3; ++col){
            homographyCv.at<double>(row, col) = homography(row, col);
        }
    }

    const cv::Mat inverseHomography = homographyCv.inv();
    const std::vector<cv::Point2f> sourceCorners = {
        {0.0f, 0.0f},
        {static_cast<float>(display.cols - 1), 0.0f},
        {static_cast<float>(display.cols - 1), static_cast<float>(display.rows - 1)},
        {0.0f, static_cast<float>(display.rows - 1)}
    };

    std::vector<cv::Point2f> warpedCorners;
    cv::perspectiveTransform(sourceCorners, warpedCorners, inverseHomography);

    float minX = warpedCorners[0].x;
    float maxX = warpedCorners[0].x;
    float minY = warpedCorners[0].y;
    float maxY = warpedCorners[0].y;
    for(const cv::Point2f& point : warpedCorners){
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }

    cv::Mat translation = cv::Mat::eye(3, 3, CV_64F);
    translation.at<double>(0, 2) = -minX;
    translation.at<double>(1, 2) = -minY;

    cv::Mat warped;
    cv::warpPerspective(
        display,
        warped,
        translation * inverseHomography,
        cv::Size(
            std::max(1, static_cast<int>(std::ceil(maxX - minX))),
            std::max(1, static_cast<int>(std::ceil(maxY - minY)))
        ),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0, 0, 0)
    );

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1200, 1000);
    cv::imshow(windowName, warped);
    cv::waitKey(0);
}

template bool saveEdgesToText<cv::Point>(
    const std::filesystem::path& outputPath,
    const std::vector<std::vector<cv::Point>>& edges
);

template bool saveEdgesToText<cv::Point2d>(
    const std::filesystem::path& outputPath,
    const std::vector<std::vector<cv::Point2d>>& edges
);

template cv::Mat renderEdgeAndCircleCenters<cv::Point>(
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point>>& edges,
    const std::vector<Circle>& centerPoints
);

template cv::Mat renderEdgeAndCircleCenters<cv::Point2d>(
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2d>>& edges,
    const std::vector<Circle>& centerPoints
);

}  // namespace camcalib::utils
