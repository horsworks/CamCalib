#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <vector>

namespace camcalib {

/** @brief 三频四步相移法的中间相位与最终绝对相位。 */
struct ThreeFrequencyPhaseResult {
    std::array<cv::Mat, 3> wrappedPhases;  ///< 三个频率对应的包裹相位。
    cv::Mat syntheticPhase23;             ///< 第二、第三频率的合成相位。
    cv::Mat syntheticPhase123;            ///< 三频级联合成相位。
    cv::Mat unwrappedPhase;                ///< 最高频率展开后的绝对相位。
};

/** @brief 三频四步相移绝对相位求解器。 */
class ThreeFrequencyFourStepPhase {
public:
    /** @brief 计算指定频率组的四步包裹相位。
     *  @param images 按三组频率、每组四帧排列的12幅 CV_32FC1 图像。
     *  @param frequencyIndex 频率组索引，取值为0、1或2。
     *  @return 范围由 atan2 决定的包裹相位图。
     */
    static cv::Mat calculateWrappedPhase(
        const std::vector<cv::Mat>& images,
        int frequencyIndex
    );

    /** @brief 计算两幅包裹相位的模 2π 差相位。
     *  @param higherFrequencyPhase 较高频率包裹相位。
     *  @param lowerFrequencyPhase 较低频率包裹相位。
     *  @return 范围为 [0, 2π) 的合成相位。
     */
    static cv::Mat calculateSyntheticPhase(
        const cv::Mat& higherFrequencyPhase,
        const cv::Mat& lowerFrequencyPhase
    );

    /** @brief 使用两级合成相位展开最高频率相位。
     *  @param syntheticPhase123 三频合成相位。
     *  @param syntheticPhase23 第二、第三频率合成相位。
     *  @param highestWrappedPhase 最高频率包裹相位。
     *  @param frequency1 最高频率。
     *  @param frequency2 中间频率。
     *  @param frequency3 最低频率。
     *  @return 展开后的最高频率绝对相位。
     */
    static cv::Mat unwrapHighestFrequency(
        const cv::Mat& syntheticPhase123,
        const cv::Mat& syntheticPhase23,
        const cv::Mat& highestWrappedPhase,
        float frequency1,
        float frequency2,
        float frequency3
    );

    /** @brief 完整执行三频四步绝对相位解算。
     *  @param images 按频率从高到低排列的12幅 CV_32FC1 图像。
     *  @param frequencies 严格从高到低排列的三个频率。
     *  @return 包裹相位、合成相位和绝对相位。
     */
    static ThreeFrequencyPhaseResult solve(
        const std::vector<cv::Mat>& images,
        const std::array<float, 3>& frequencies
    );
};

}  // namespace camcalib
