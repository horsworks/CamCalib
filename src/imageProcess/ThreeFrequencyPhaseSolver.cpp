#include "imageProcess/ThreeFrequencyPhaseSolver.h"

#include "imageProcess/PhaseShiftDecoder.h"

#include <cmath>
#include <cstddef>

namespace camcalib {
namespace {

constexpr float kTwoPi = 6.28318530717958647692f;

constexpr float kUnwrapRoundingBias = 0.2f;

void checkFloatPhaseImage(const cv::Mat& image){
    CV_Assert(!image.empty());
    CV_Assert(image.type() == CV_32FC1);
}

void checkSameSizeAndType(const cv::Mat& first, const cv::Mat& second){
    checkFloatPhaseImage(first);
    checkFloatPhaseImage(second);
    CV_Assert(first.size() == second.size());
}

std::vector<cv::Mat> getFrequencyImages(
    const std::vector<cv::Mat>& images,
    int frequencyIndex,
    int phaseSteps
){
    const std::size_t begin =
        static_cast<std::size_t>(frequencyIndex * phaseSteps);
    const std::size_t end = begin + static_cast<std::size_t>(phaseSteps);

    return std::vector<cv::Mat>(
        images.begin() + static_cast<std::ptrdiff_t>(begin),
        images.begin() + static_cast<std::ptrdiff_t>(end)
    );
}

}  // namespace

cv::Mat ThreeFrequencyPhaseSolver::calculateSyntheticPhase(
    const cv::Mat& higherFrequencyPhase,
    const cv::Mat& lowerFrequencyPhase
){
    checkSameSizeAndType(higherFrequencyPhase, lowerFrequencyPhase);
    cv::Mat syntheticPhase(higherFrequencyPhase.size(), CV_32FC1);

    for(int row = 0; row < syntheticPhase.rows; ++row){
        const float* highPtr = higherFrequencyPhase.ptr<float>(row);
        const float* lowPtr = lowerFrequencyPhase.ptr<float>(row);
        float* resultPtr = syntheticPhase.ptr<float>(row);

        for(int col = 0; col < syntheticPhase.cols; ++col){
            float phaseDifference = highPtr[col] - lowPtr[col];
            if(phaseDifference < 0.0f){
                phaseDifference += kTwoPi;
            }
            resultPtr[col] = phaseDifference;
        }
    }

    return syntheticPhase;
}

cv::Mat ThreeFrequencyPhaseSolver::unwrapHighestFrequency(
    const cv::Mat& syntheticPhase123,
    const cv::Mat& syntheticPhase23,
    const cv::Mat& highestWrappedPhase,
    float frequency1,
    float frequency2,
    float frequency3
){
    checkSameSizeAndType(syntheticPhase123, syntheticPhase23);
    checkSameSizeAndType(syntheticPhase123, highestWrappedPhase);
    CV_Assert(frequency1 > frequency2 && frequency2 > frequency3);

    const float referenceLength = static_cast<float>(highestWrappedPhase.cols);
    const float period1 = referenceLength / frequency1;
    const float period2 = referenceLength / frequency2;
    const float period3 = referenceLength / frequency3;
    const float period12 = period1 * period2 / (period2 - period1);
    const float period23 = period2 * period3 / (period3 - period2);
    const float period123 = period12 * period23 / (period23 - period12);

    cv::Mat unwrappedPhase23(syntheticPhase123.size(), CV_32FC1);
    cv::Mat unwrappedPhase1(syntheticPhase123.size(), CV_32FC1);

    for(int row = 0; row < syntheticPhase123.rows; ++row){
        for(int col = 0; col < syntheticPhase123.cols; ++col){
            const float phase123 = syntheticPhase123.at<float>(row, col);
            const float phase23 = syntheticPhase23.at<float>(row, col);
            unwrappedPhase23.at<float>(row, col) =
                phase23 + kTwoPi * static_cast<float>(std::round(
                    (phase123 * period123 / period23 - phase23 +
                     kUnwrapRoundingBias) / kTwoPi
                ));
        }
    }

    for(int row = 0; row < syntheticPhase123.rows; ++row){
        for(int col = 0; col < syntheticPhase123.cols; ++col){
            const float phase1 = highestWrappedPhase.at<float>(row, col);
            const float phase23 = unwrappedPhase23.at<float>(row, col);
            unwrappedPhase1.at<float>(row, col) =
                phase1 + kTwoPi * static_cast<float>(std::round(
                    (phase23 * period23 / period1 - phase1 +
                     kUnwrapRoundingBias) / kTwoPi
                ));
        }
    }

    return unwrappedPhase1;
}

ThreeFrequencyPhaseResult ThreeFrequencyPhaseSolver::solve(
    const std::vector<cv::Mat>& images,
    const std::array<float, 3>& frequencies,
    int phaseSteps
){
    CV_Assert(phaseSteps >= 3);

    const std::size_t expectedImageCount =
        frequencies.size() * static_cast<std::size_t>(phaseSteps);
    CV_Assert(images.size() == expectedImageCount);

    ThreeFrequencyPhaseResult result;
    for(int frequencyIndex = 0; frequencyIndex < 3; ++frequencyIndex){
        result.wrappedPhases[frequencyIndex] = PhaseShiftDecoder::decode(
            getFrequencyImages(images, frequencyIndex, phaseSteps)
        );
    }

    const cv::Mat syntheticPhase12 = calculateSyntheticPhase(
        result.wrappedPhases[0],
        result.wrappedPhases[1]
    );
    result.syntheticPhase23 = calculateSyntheticPhase(
        result.wrappedPhases[1],
        result.wrappedPhases[2]
    );
    result.syntheticPhase123 = calculateSyntheticPhase(
        syntheticPhase12,
        result.syntheticPhase23
    );
    result.unwrappedPhase = unwrapHighestFrequency(
        result.syntheticPhase123,
        result.syntheticPhase23,
        result.wrappedPhases[0],
        frequencies[0],
        frequencies[1],
        frequencies[2]
    );

    return result;
}

}  // namespace camcalib
