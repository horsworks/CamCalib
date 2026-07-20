

#include "calibration/CustomCalibrator.h"

namespace camcalib{

cv::Mat CustomCalibrator::initializeIntrinsics(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2d>& imagePoints
) const{

    // 对objectPoints 与 imagePoints 进行归一化

    const int pointsNumb =  imagePoints.size();

    // step1: 使用点对构造方程组 设方程AX = 0
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * pointsNumb, 9);

    for(int i = 0; i < pointsNumb; ++i){
        const cv::Point3f& objPoint = objectPoints[i];
        const cv::Point2d& imgPoint = imagePoints[i];

        double X = static_cast<double>(objPoint.x); 
        double Y = static_cast<double>(objPoint.y);
        double u = imgPoint.x;
        double v = imgPoint.y;

        const int row0 = 2 * i;
        const int row1 = 2 * i + 1;

        A(row0, 0) = X;
        A(row0, 1) = Y;
        A(row0, 2) = 1.0;
        A(row0, 3) = 0.0;
        A(row0, 4) = 0.0;
        A(row0, 5) = 0.0;
        A(row0, 6) = -u * X;
        A(row0, 7) = -u * Y;
        A(row0, 8) = -u;

        A(row1, 0) = 0.0;
        A(row1, 1) = 0.0;
        A(row1, 2) = 0.0;
        A(row1, 3) = X;
        A(row1, 4) = Y;
        A(row1, 5) = 1.0;
        A(row1, 6) = -v * X;
        A(row1, 7) = -v * Y;
        A(row1, 8) = -v;

        // 还可以写成 A（rows） << ......;

    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    // v的最后一列对应最小奇异值
    Eigen::VectorXd h = svd.matrixV().col(8);

    Eigen::Matrix3d H;

    H << h(0), h(1), h(2),
         h(3), h(4), h(5),
         h(6), h(7), h(8);

    if(std::abs(H(2, 2)) > 1e-12){   // 归一化
        H /= H(2, 2);  
    }else{
        H /= H.norm(); 
    }

}

}

