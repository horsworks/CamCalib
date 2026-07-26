#pragma once

#include <opencv2/opencv.hpp>

namespace camcalib {

    /** @brief 手动实现的 Canny 边缘检测器。 */
    class CannyDetecter {
    public:
        /** @brief 创建 Canny 边缘检测器。
         *  @param kernel_size 高斯核边长。
         *  @param sigma_ 高斯核标准差。
         *  @param low_threshold 双阈值中的低阈值。
         *  @param high_threshold 双阈值中的高阈值。
         */
        CannyDetecter(int kernel_size = 5, double sigma_ = 1.0,double low_threshold = 50, double high_threshold = 150);

        /** @brief 对输入图像执行完整 Canny 流程。
         *  @param image 输入灰度图像。
         *  @return 8位单通道边缘图。
         */
        cv::Mat detect(const cv::Mat& image);

    private:
        int kernel_size_;         ///< 高斯模糊核边长。
        double sigma_;            ///< 高斯模糊标准差。
        double low_threshold_;    ///< 边缘连接低阈值。
        double high_threshold_;   ///< 强边缘高阈值。

        /** @brief 生成归一化二维高斯卷积核。 */
        cv::Mat getConvolutionKernel(cv::Size kernel_size, double sigma);

        /** @brief 使用指定卷积核平滑图像。 */
        cv::Mat applyGaussianBlur(const cv::Mat& image,const cv::Mat& kernel);

        /** @brief 计算梯度幅值和方向。 */
        void computeGradient(const cv::Mat& image, cv::Mat& gradient_magnitude, cv::Mat& gradient_direction);

        /** @brief 对梯度幅值执行非极大值抑制。 */
        cv::Mat nonMaximumSuppression(const cv::Mat& gradient_magnitude, const cv::Mat& gradient_direction);

        /** @brief 执行双阈值分类和弱边缘连接。 */
        cv::Mat thresholdAndLinkEdges(const cv::Mat& non_max_suppressed);
    };

}  // namespace camcalib
