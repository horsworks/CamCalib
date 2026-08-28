#include "imageProcess/PhaseShiftDecoder.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

namespace {

constexpr float kTwoPi = 6.28318530717958647692f;

std::vector<cv::Mat> generatePhaseImages(
    const cv::Mat& phase,
    int phaseSteps
) {
    std::vector<cv::Mat> images;
    images.reserve(phaseSteps);

    for (int step = 0; step < phaseSteps; ++step) {
        const float phaseShift =
            kTwoPi * static_cast<float>(step) /
            static_cast<float>(phaseSteps);

        cv::Mat image(phase.size(), CV_32FC1);

        for (int row = 0; row < phase.rows; ++row) {
            for (int col = 0; col < phase.cols; ++col) {
                const float value = phase.at<float>(row, col);

                image.at<float>(row, col) =
                    100.0f + 50.0f * std::cos(value + phaseShift);
            }
        }

        images.push_back(std::move(image));
    }

    return images;
}

float wrappedDifference(float first, float second) {
    return std::atan2(
        std::sin(first - second),
        std::cos(first - second)
    );
}

bool checkPhaseEqual(
    const cv::Mat& first,
    const cv::Mat& second,
    float tolerance
) {
    for (int row = 0; row < first.rows; ++row) {
        for (int col = 0; col < first.cols; ++col) {
            const float error = std::abs(
                wrappedDifference(
                    first.at<float>(row, col),
                    second.at<float>(row, col)
                )
            );

            if (error > tolerance) {
                return false;
            }
        }
    }

    return true;
}

}  // namespace

int main() {
    const cv::Mat expectedPhase = (
        cv::Mat_<float>(2, 3) <<
        -2.5f, -1.0f, 0.0f,
         0.5f,  1.5f, 2.5f
    );

    // 验证通用 N 步公式能够正确恢复相位。
    const std::vector<cv::Mat> fourStepImages =
        generatePhaseImages(expectedPhase, 4);

    const cv::Mat decodedFourStep =
        camcalib::PhaseShiftDecoder::decode(fourStepImages);

    if (!checkPhaseEqual(decodedFourStep, expectedPhase, 1e-4f)) {
        std::cerr << "Four-step phase decoding failed." << std::endl;
        return 1;
    }

    // 验证 N=4 时与原项目四步公式完全一致。
    cv::Mat legacyFourStep(expectedPhase.size(), CV_32FC1);

    for (int row = 0; row < expectedPhase.rows; ++row) {
        for (int col = 0; col < expectedPhase.cols; ++col) {
            legacyFourStep.at<float>(row, col) = std::atan2(
                fourStepImages[3].at<float>(row, col) -
                    fourStepImages[1].at<float>(row, col),
                fourStepImages[0].at<float>(row, col) -
                    fourStepImages[2].at<float>(row, col)
            );
        }
    }

    if (!checkPhaseEqual(decodedFourStep, legacyFourStep, 1e-4f)) {
        std::cerr << "Four-step compatibility test failed." << std::endl;
        return 1;
    }

    // 再验证一次真正的非四步情况。
    const std::vector<cv::Mat> fiveStepImages =
        generatePhaseImages(expectedPhase, 5);

    const cv::Mat decodedFiveStep =
        camcalib::PhaseShiftDecoder::decode(fiveStepImages);

    if (!checkPhaseEqual(decodedFiveStep, expectedPhase, 1e-4f)) {
        std::cerr << "Five-step phase decoding failed." << std::endl;
        return 1;
    }

    std::cout << "PhaseShiftDecoder tests passed." << std::endl;
    return 0;
}