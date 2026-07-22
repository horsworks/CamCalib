

#include "calibration/CustomCalibrator.h"

namespace camcalib{


// 对点集进行归一化
std::pair<std::vector<cv::Point2d>, Eigen::Matrix3d> CustomCalibrator::normalizeObjectCoordinates(
        const std::vector<cv::Point2d>& coordinates
) const {

    if (coordinates.empty()) {
        throw std::invalid_argument("输入点集不能为空");
    }

    const int pointCount = coordinates.size();

    // 计算质心

    double centerX = 0.0;
    double centerY = 0.0;

    for(const auto& point : coordinates){      

        centerX += point.x;
        centerY += point.y;
    }

    centerX /= static_cast<double>(pointCount);
    centerY /= static_cast<double>(pointCount);

    // 计算平面内的欧式距离

    double meanDistance = 0.0;

    for (const auto& point : coordinates){

        const double dx = point.x - centerX;
        const double dy = point.y - centerY;

        meanDistance += std::hypot(dx, dy);   // sqrt(dx^2, dy^2)
    }

    meanDistance /= static_cast<double>(pointCount);

    if (meanDistance < 1e-12) {
        throw std::runtime_error("物点集合发生退化");
    }

    // 缩放至平均距离

    const double scale = std::sqrt(2.0) / meanDistance;

    Eigen::Matrix3d transform;

    transform <<                        //  放缩矩阵
        scale, 0.0, -scale * centerX,
        0.0, scale, -scale * centerY,
        0.0, 0.0, 1.0;

    // 输出归一化后的坐标

    std::vector<cv::Point2d>  normalizedPoints;
    normalizedPoints.reserve(pointCount);

    for(const auto& point : coordinates){

        const double normalizedX = scale * (point.x - centerX);
        const double normalizedY = scale * (point.y - centerY);

        const cv::Point2d p (normalizedX, normalizedY);
        normalizedPoints.push_back(p);
    }

    return {std::move(normalizedPoints), transform};

}

// 数据类型转换
std::vector<cv::Point2d> CustomCalibrator::point3f2point2d(
        const std::vector<cv::Point3f>& worldPoints
    ) const{

    const int pointCount = static_cast<int>(worldPoints.size());

    std::vector<cv::Point2d> worldPoints2d;
    worldPoints2d.reserve(pointCount);

    for(const auto& point : worldPoints){

        worldPoints2d.push_back(
            cv::Point2d(static_cast<double>(point.x), static_cast<double>(point.y))
        );
    }

    return worldPoints2d;

}


Eigen::Matrix3d CustomCalibrator::estimateHomography(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2d>& imagePoints
) const{

    // 对objectPoints 与 imagePoints 进行归一化， 为什么要做归一化呢， 为什么平均距离为sqrt(2)呢
    // 归一化
    std::vector<cv::Point2d> objectPoints2d;
    objectPoints2d = point3f2point2d(objectPoints);

    auto [noramlizedObjectPoints2d, transformObject] = normalizeObjectCoordinates(objectPoints2d);
    auto [normalizeImagePoints, transformImgae] = normalizeObjectCoordinates(imagePoints);

    // 计算变换矩阵
    const int pointsNumb =  imagePoints.size();

    // step1: 使用点对构造方程组 设方程AX = 0
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * pointsNumb, 9);

    for(int i = 0; i < pointsNumb; ++i){
        const cv::Point2d& objPoint = noramlizedObjectPoints2d[i];
        const cv::Point2d& imgPoint = normalizeImagePoints[i];

        double X = static_cast<double>(objPoint.x); 
        double Y = static_cast<double>(objPoint.y);
        double u = imgPoint.x;
        double v = imgPoint.y;

        const int row0 = 2 * i;
        const int row1 = 2 * i + 1;
 
        A.row(row0) << X, Y, 1,    // Eigen::vector 可以使用使用这种方式
                   0.0, 0.0, 0.0,
                   -u * X, -u * Y, -u;

        A.row(row1) << 0.0, 0.0, 0.0,
                       X, Y, 1.0,
                       -v * X, -v * Y, -v;

    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    // v的最后一列对应最小奇异值
    Eigen::VectorXd h = svd.matrixV().col(8);

    Eigen::Matrix3d HNormalized;

    HNormalized << h(0), h(1), h(2),   // 归一化尺度下的H
                   h(3), h(4), h(5),
                   h(6), h(7), h(8);

    // 进行反归一化
    Eigen::Matrix3d H = transformImgae.inverse() * HNormalized * transformObject;

    // 固定整体尺度
    if(std::abs(H(2, 2)) > 1e-12){   // 防止出现H(2, 2) = 0的情况
        H /= H(2,2);
    }else{

        const double norm = H.norm();
    }

    return H;

}

}

