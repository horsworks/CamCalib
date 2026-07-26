

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

// 计算所有位姿的Homo矩阵
std::vector<Eigen::Matrix3d> CustomCalibrator::estimateAllPoseHomography(
    const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const std::vector<std::vector<cv::Point2d>>& imagePoints
) const{

    if(objectPoints.size() != imagePoints.size()){

        return {};
    }

    const int matrixCount = imagePoints.size();

    std::vector<Eigen::Matrix3d> matrixVector;

    for(int i = 0; i < matrixCount; ++i){

        Eigen::Matrix3d homo =  estimateHomography(objectPoints.at(i), imagePoints.at(i));

        matrixVector.push_back(homo);

    }

    return matrixVector;

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
    // v的最后一列对应最小奇异值   为什么呢
    Eigen::VectorXd h = svd.matrixV().col(8);

    Eigen::Matrix3d HNormalized;

    HNormalized << h(0), h(1), h(2),   // 归一化尺度下的H
                   h(3), h(4), h(5),
                   h(6), h(7), h(8);

    // 进行反归一化   反归一化是为了将H变换到真实的空间尺度中
    Eigen::Matrix3d H = transformImgae.inverse() * HNormalized * transformObject;

    // 固定 H 尺度   H的自由度只有8个  H = nH  
    if(std::abs(H(2, 2)) > 1e-12){   
        H /= H(2,2);
    }else{

        const double norm = H.norm(); // 防止出现H(2, 2) = 0的情况
    }

    return H;

}

// 该矩阵的构造详情见张氏标定推导
Eigen::Matrix<double, 6, 1> CustomCalibrator::makeV(
    const Eigen::Matrix3d& H,
    int i, int j 
) const{

    Eigen::Matrix<double, 6, 1> v;

    v << H(0, i) * H(0, j),
         H(0, i) * H(1, j) + H(1, j) * H(0, j),
         H(1, i) * H(1, j),
         H(2, i) * H(0, j) + H(0, i) * H(2, j),
         H(2, i) * H(1, j) + H(1, i) * H(2, j),
         H(2, i) * H(2, j);


    return v;
}

// 通过homo矩阵计算内外参数初值
cv::Mat CustomCalibrator::estimateIntrinsics(
    const std::vector<Eigen::Matrix3d>& homographies
) const{

    CV_Assert(homographies.size() >= 3);

    Eigen::MatrixXd V(     // size = homographies.size() *2  6列 
        static_cast<Eigen::Index>(homographies.size() *2), 6
    );

    // 构造矩阵V    方程为VB = 0    其中B为K^-t * K
    for(size_t k = 0; k < homographies.size(); ++k){

        const Eigen::Matrix3d& H = homographies[k];

        // h1^T B h2 = 0    旋转矩阵 列向量正交
        V.row(static_cast<Eigen::Index>(2 * k)) = makeV(H, 0, 1).transpose();

        // h1^T B h1 - h2^T B h2 = 0    ||r1|| = ||r2|| 单位向量
        V.row(static_cast<Eigen::Index>(2 * k + 1)) = 
        (makeV(H, 0, 0) - makeV(H, 1, 1)).transpose();
    }

    // 求 Vb = 0 的最小二乘齐次解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullV);

    Eigen::VectorXd b = svd.matrixV().col(5);    // b 只与内参有关

    // 计算内参参数 
    /*
     * b = [B11, B12, B22, B13, B23, B33]^T
     *
     * 因为 Vb = 0 是齐次方程，所以 b 和 -b 等价。
     * 尝试用简单条件判断是否需要翻转符号。
     */



}



}

