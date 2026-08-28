#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <vector>

namespace camcalib {

/** @brief 三频相移法的中间相位与最终绝对相位。 */
struct ThreeFrequencyPhaseResult {
    std::array<cv::Mat, 3> wrappedPhases;  ///< 三个频率对应的包裹相位。
    cv::Mat syntheticPhase23;             ///< 第二、第三频率的合成相位。
    cv::Mat syntheticPhase123;            ///< 三频级联合成相位。
    cv::Mat unwrappedPhase;                ///< 最高频率展开后的绝对相位。
};

/** @brief 三频 + 通用等间隔 N 步相移绝对相位求解器。 */
class ThreeFrequencyPhaseSolver {
public:
    /** @brief 计算两幅包裹相位的模 2pi 差相位。 */
    static cv::Mat calculateSyntheticPhase(
        const cv::Mat& higherFrequencyPhase,
        const cv::Mat& lowerFrequencyPhase
    );

    /** @brief 使用两级合成相位展开最高频率相位。 */
    static cv::Mat unwrapHighestFrequency(
        const cv::Mat& syntheticPhase123,
        const cv::Mat& syntheticPhase23,
        const cv::Mat& highestWrappedPhase,
        float frequency1,
        float frequency2,
        float frequency3
    );

    /**
     * @brief 完整执行三频 N 步绝对相位解算。
     * @param images 按频率分组排列的 3*N 张 CV_32FC1 图像：
     *        [f0_step0 ... f0_stepN-1,
     *         f1_step0 ... f1_stepN-1,
     *         f2_step0 ... f2_stepN-1]。
     * @param frequencies 严格从高到低排列的三个频率。
     * @param phaseSteps 每个频率的相移步数 N，至少为 3。
     */
    static ThreeFrequencyPhaseResult solve(
        const std::vector<cv::Mat>& images,
        const std::array<float, 3>& frequencies,
        int phaseSteps
    );
};

}  // namespace camcalib
