#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <vector>

namespace camcalib {

struct ThreeFrequencyPhaseResult {
    std::array<cv::Mat, 3> wrappedPhases;
    cv::Mat syntheticPhase23;
    cv::Mat syntheticPhase123;
    cv::Mat unwrappedPhase;
};

class ThreeFrequencyFourStepPhase {
public:
    static cv::Mat calculateWrappedPhase(
        const std::vector<cv::Mat>& images,
        int frequencyIndex
    );

    static cv::Mat calculateSyntheticPhase(
        const cv::Mat& higherFrequencyPhase,
        const cv::Mat& lowerFrequencyPhase
    );

    static cv::Mat unwrapHighestFrequency(
        const cv::Mat& syntheticPhase123,
        const cv::Mat& syntheticPhase23,
        const cv::Mat& highestWrappedPhase,
        float frequency1,
        float frequency2,
        float frequency3
    );

    static ThreeFrequencyPhaseResult solve(
        const std::vector<cv::Mat>& images,
        const std::array<float, 3>& frequencies
    );
};

}  // namespace camcalib
