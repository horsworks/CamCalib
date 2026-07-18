#include "imageProcess/EdgeProcessing.h"

#include <algorithm>
#include <complex>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace camcalib::image {

static cv::Mat buildGrayImage(const cv::Mat& image){
    cv::Mat grayImage;
    if(image.channels() == 1){
        grayImage = image;
    }else{
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    }

    return grayImage;
}

static cv::Mat buildBinaryImage(const cv::Mat& image){
    cv::Mat grayImage = buildGrayImage(image);

    cv::Mat binaryImage;
    cv::threshold(grayImage, binaryImage, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    return binaryImage;
}

static std::vector<std::vector<cv::Point>> filterCircularContours(
    const std::vector<std::vector<cv::Point>>& contours,
    const DetectorConfig& detectorConfig
){
    std::vector<std::vector<cv::Point>> pointCountFilteredContours;

    for(const std::vector<cv::Point>& contour : contours){
        if(static_cast<int>(contour.size()) > detectorConfig.minContourPoints){
            pointCountFilteredContours.push_back(contour);
        }
    }

    std::vector<std::vector<cv::Point>> circularContours;
    for(const std::vector<cv::Point>& contour : pointCountFilteredContours){
        if(contour.empty()){
            continue;
        }

        int minX = contour[0].x;
        int maxX = contour[0].x;
        int minY = contour[0].y;
        int maxY = contour[0].y;

        for(const cv::Point& point : contour){
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }

        const int width = maxX - minX + 1;
        const int height = maxY - minY + 1;
        const int longAxis = std::max(width, height);
        const int shortAxis = std::min(width, height);
        const double axisRatio = static_cast<double>(longAxis) / static_cast<double>(shortAxis);

        if(axisRatio <= detectorConfig.maxAxisRatio){
            circularContours.push_back(contour);
        }
    }

    return circularContours;
}

static std::vector<std::vector<std::vector<cv::Point2d>>> refineEdgesToSubPixel(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges,
    cv::Size kernelSize
){
    std::vector<std::vector<std::vector<cv::Point2d>>> subPixelEdges;
    subPixelEdges.reserve(images.size());
    const int radius = kernelSize.width / 2;

    for(size_t imageIndex = 0; imageIndex < images.size(); ++imageIndex){
        const cv::Mat& grayImage = images[imageIndex];
        std::vector<std::vector<cv::Point2d>> imageSubPixelEdges;
        imageSubPixelEdges.reserve(pixelEdges[imageIndex].size());

        for(size_t contourIndex = 0; contourIndex < pixelEdges[imageIndex].size(); ++contourIndex){
            std::vector<cv::Point2d> contourSubPixelEdges;
            contourSubPixelEdges.reserve(pixelEdges[imageIndex][contourIndex].size());

            for(const cv::Point& pixelPoint : pixelEdges[imageIndex][contourIndex]){
                std::complex<double> a11(0.0, 0.0);
                double a20 = 0.0;

                for(int dy = -radius; dy <= radius; ++dy){
                    for(int dx = -radius; dx <= radius; ++dx){
                        const double x = static_cast<double>(dx) / radius;
                        const double y = static_cast<double>(dy) / radius;
                        if(x * x + y * y > 1.0){
                            continue;
                        }

                        const double intensity = static_cast<double>(
                            grayImage.at<uchar>(pixelPoint.y + dy, pixelPoint.x + dx)
                        );
                        a11 += intensity * std::complex<double>(x, -y);
                        a20 += intensity * (2.0 * x * x + 2.0 * y * y - 1.0);
                    }
                }

                a11 *= 2.0 / CV_PI;
                a20 *= 3.0 / CV_PI;

                const double absA11 = std::abs(a11);
                if(absA11 < 1e-6){
                    continue;
                }

                const double phi = std::atan2(a11.imag(), a11.real());
                double offsetScale = -2.0 * a20 / (3.0 * absA11);
                if(std::abs(offsetScale) > 1.0){
                    continue;
                }

                contourSubPixelEdges.emplace_back(
                    pixelPoint.x + offsetScale * radius * std::cos(phi),
                    pixelPoint.y + offsetScale * radius * std::sin(phi)
                );
            }

            imageSubPixelEdges.push_back(contourSubPixelEdges);
        }

        subPixelEdges.push_back(imageSubPixelEdges);
    }

    return subPixelEdges;
}

std::vector<std::vector<cv::Point>> cannyDetect(const cv::Mat& gray){

    if (gray.empty())
    {
        std::cerr << "Input image is empty!" << std::endl;
        return {};
    }

    cv::Mat blurImg;
    cv::GaussianBlur(gray, blurImg, cv::Size(5, 5), 1.0);

    // 黑底白圆、白底黑圆都可以用 Canny，因为它看的是梯度
    double lowThreshold = 50.0;
    double highThreshold = 150.0;
    cv::Mat edges;

    cv::Canny(blurImg,edges,lowThreshold,highThreshold, 3,true);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(edges,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE );

    return contours;

}

std::vector<std::vector<std::vector<cv::Point>>> detectEdges(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& detectorConfig
){
    std::vector<std::vector<std::vector<cv::Point>>> allImageContours;
    allImageContours.reserve(images.size());

    for(const cv::Mat& image : images){
        cv::Mat binaryImage = buildBinaryImage(image);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binaryImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        allImageContours.push_back(filterCircularContours(contours, detectorConfig));
    }

    return allImageContours;
}

// 使用梯度检测整像素边缘
std::vector<std::vector<std::vector<cv::Point>>> detectEdgesGradient(
    const std::vector<cv::Mat>& images,
    const DetectorConfig& detectorConfig
){
   
    std::vector<std::vector<std::vector<cv::Point>>> allImageContours;
    allImageContours.reserve(images.size());


    for(const cv::Mat& image : images){
        const cv::Mat grayImage = buildGrayImage(image);
        const std::vector<std::vector<cv::Point>> cannyEdges = cannyDetect(grayImage);

        const std::vector<std::vector<cv::Point>> filteredCannyEdges =
            filterCircularContours(cannyEdges, detectorConfig);

        allImageContours.push_back(filteredCannyEdges);
    }


    return allImageContours;
}


// 从圆心做射线， 采集射线上的梯度最大点 拟合为亚像素坐标

std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges_ray(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
){


    // 



    (void)images;
    return toSubPixelContours(pixelEdges);
}

std::vector<std::vector<std::vector<cv::Point2d>>> toSubPixelContours(   // 只是类型转换
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
){
    std::vector<std::vector<std::vector<cv::Point2d>>> contours;
    contours.reserve(pixelEdges.size());
    for(const auto& imageContours : pixelEdges){
        std::vector<std::vector<cv::Point2d>> imageSubPixelContours;
        imageSubPixelContours.reserve(imageContours.size());
        for(const auto& contour : imageContours){
            std::vector<cv::Point2d> convertedContour;
            convertedContour.reserve(contour.size());
            for(const cv::Point& point : contour){
                convertedContour.emplace_back(point.x, point.y);
            }
            imageSubPixelContours.push_back(std::move(convertedContour));
        }
        contours.push_back(std::move(imageSubPixelContours));
    }
    return contours;
}

std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
){

    return refineEdgesToSubPixel(images, pixelEdges, cv::Size(9, 9));
}

void showCannyEdges(const cv::Mat& image, const std::string& windowName){
    if(image.empty()){
        std::cerr << "Input image is empty!" << std::endl;
        return;
    }

    const cv::Mat grayImage = buildGrayImage(image);
    const std::vector<std::vector<cv::Point>> cannyContours = cannyDetect(grayImage);

    cv::Mat display;
    if(image.channels() == 1){
        cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    }else{
        display = image.clone();
    }

    cv::drawContours(display, cannyContours, -1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1200, 1000);
    cv::imshow(windowName, display);
    cv::waitKey(0);
}

// blob检测


}  // namespace camcalib::image
