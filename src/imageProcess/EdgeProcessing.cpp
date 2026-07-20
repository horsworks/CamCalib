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


//  亚像素边缘提取
bool computeGradientField(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag){

    cv::Mat grayF, smooth;

    img.convertTo(grayF, CV_32F, 1.0 /255.0);

    // cv::Size(0, 0) 表示根据sigma 自适应size大小
    cv::GaussianBlur(grayF, smooth, cv::Size(0, 0), 1.0, 1.0);

    cv::Scharr(smooth, gx, CV_32F, 1, 0);      // shcarr 算子计算梯度
    cv::Scharr(smooth, gy, CV_32F, 0, 1);

    cv::magnitude(gx, gy, mag);

    return true;
}


bool bilinearSample(const cv::Mat& image, double x, double y, double& value){

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));

    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if(x0 < 0 || y0 < 0 || x1 >= image.cols || y1 >= image.rows){
        return false;
    }

    const double dx = x - x0;
    const double dy = y - y0;

    const double v00 = image.at<float>(y0, x0);
    const double v10 = image.at<float>(y0, x1);
    const double v01 = image.at<float>(y1, x0);
    const double v11 = image.at<float>(y1, x1);

    value = (1.0 - dx) * (1.0 -dy) * v00 +
            dx * (1.0 - dy) *v10 +
            (1.0 - dx ) * dy * v01 +
            dx * dy * v11;

    return true;
}

// 单个像素点的亚像素细化
bool refineOneEdgePoint(
    const cv::Point& pixelPoint,
    const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& mag,
    cv::Point2d& subpixelPoint,
    int searchRadius,
    double minGradient ){

    CV_Assert(gx.type()  == CV_32F);
    CV_Assert(gy.type()  == CV_32F);
    CV_Assert(mag.type() == CV_32F);

    const int x = pixelPoint.x;
    const int y = pixelPoint.y;

    const int margin = searchRadius + 2;

    if (x < margin ||
        y < margin ||
        x >= mag.cols - margin ||
        y >= mag.rows - margin) {
        return false;
    }

    // 1. 整像素边缘点处的初始梯度法向

    const double gx0 = gx.at<float>(y, x);
    const double gy0 = gy.at<float>(y, x);
    // hypot = sqrt(x^2 + y^2)  对单个点进行计算
    const double gradient0 = std::hypot(gx0, gy0); 

    if (gradient0 < minGradient) {
        return false;
    }

    double nx = gx0 / gradient0;    // 整像素的法线方向
    double ny = gy0 / gradient0;

    // 2. 沿初始法向寻找梯度幅值的粗峰值
    int bestStep = 0;
    double bestMagnitude = - 1.0;

    for(int step = -searchRadius; step <= searchRadius; ++step ){
 
        double value = 0.0;

        bool ret = bilinearSample(mag, x + step * nx, y + step *ny, value);
        if (!ret){
            continue;
        }
        
        if(value > bestMagnitude){
            bestMagnitude = value;
            bestStep = step;
        }

    }

    // 峰值位于搜索区端点，可能没有覆盖真实峰值
    if(bestStep == -searchRadius || bestStep == searchRadius){
        return false;
    }

    const double peakX = x + bestStep * nx;   // 第一次搜索到的峰值亚像素坐标
    const double peakY = y + bestStep * ny;
    
    // 3. 在粗峰值位置重新计算法向
    double gxPeak = 0.0;
    double gyPeak = 0.0;

    bool retGx = bilinearSample(gx, peakX, peakY, gxPeak);
    bool retGy = bilinearSample(gy, peakX, peakY, gyPeak);
    if (!retGx || !retGy) {
        return false;
    }

    const double peakGradient = std::hypot(gxPeak, gyPeak);  

    if(peakGradient < minGradient){ 
        return false;
    }

    nx = gxPeak / peakGradient; // 重新计算的法线方向
    ny = gyPeak / peakGradient;

    // 4. 沿更新后的法向，在粗峰值两侧采样
    double gm = 0.0;
    double g0 = 0.0;
    double gp = 0.0;

    if (!bilinearSample(mag, peakX - nx, peakY - ny, gm) ||
        !bilinearSample(mag, peakX, peakY, g0) ||
        !bilinearSample(mag, peakX + nx, peakY + ny, gp)) {
        return false;
    }

     // 粗峰值必须是法向上的局部极大值
    if (g0 < gm || g0 < gp) {
        return false;
    }

    // 5. 三点抛物线插值  常用的拟合方式  就是三个点拟合一个二次抛物线
    const double denominator = gm - 2.0 * g0 + gp;

    // 极大值附近抛物线应开口向下
    if (denominator >= -1e-12) {
        return false;
    }

    const double delta = 0.5 * (gm - gp) / denominator;

    if (!std::isfinite(delta) ||
        std::abs(delta) > 0.5) {
        return false;
    }

    subpixelPoint.x = peakX + delta * nx;
    subpixelPoint.y = peakY + delta * ny;

    return true;
}

// 单个轮廓
std::vector<cv::Point2d> refineOneContour(
    const std::vector<cv::Point>& pixelContour,
    const cv::Mat& gx,
    const cv::Mat& gy,
    const cv::Mat& mag){

    std::vector<cv::Point2d> subpixelContour;
    subpixelContour.reserve(pixelContour.size());

    for (const cv::Point& pixelPoint : pixelContour){
        cv::Point2d subpixelPoint;

        if(refineOneEdgePoint(pixelPoint, gx, gy, mag, subpixelPoint)){
            subpixelContour.push_back(subpixelPoint);
        }

    }

    return subpixelContour;

}

std::vector<std::vector<std::vector<cv::Point2d>>> detectSubPixelEdges_Canny(
    const std::vector<cv::Mat>& images,
    const std::vector<std::vector<std::vector<cv::Point>>>& pixelEdges
){
   
    if(images.size() != pixelEdges.size()){
        throw std::invalid_argument("images and pixelEdges size mismatch");
    }

    std::vector<std::vector<std::vector<cv::Point2d>>> result;
    result.reserve(images.size());

    for(std::size_t i = 0; i < images.size(); ++i){

        cv::Mat gx, gy, mag;

        if(!computeGradientField(images[i], gx, gy, mag)){
            result.emplace_back();    // 保持对应关系
            continue;
        }

        std::vector<std::vector<cv::Point2d>> imageResult;
        imageResult.reserve(pixelEdges[i].size());

        for(const auto& contour : pixelEdges[i]){

            imageResult.push_back(refineOneContour(contour, gx, gy, mag));
        }

        result.push_back(std::move(imageResult));   // 用move的好处  减少复制，直接移动资源

    }

    return result;
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

    // return refineEdgesToSubPixel(images, pixelEdges, cv::Size(9, 9));
    return detectSubPixelEdges_Canny(images, pixelEdges);
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
