#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace camcalib {

    /** @brief 等间隔 N 步相移包裹相位解码器。 */
    class PhaseShiftDecoder {
    public:
        /**
         * @brief 根据同一频率下的 N 张等间隔相移图计算包裹相位。
         * @param phaseImages 按相移顺序排列的 N 张 CV_32FC1 图像。
         *        第 k 张对应相移 delta_k = 2*pi*k/N。
         * @return 范围为 [-pi, pi] 的 CV_32FC1 包裹相位图。
         */
        static cv::Mat decode(const std::vector<cv::Mat>& phaseImages);
    };

}  // namespace camcalib
