#include "imageProcess/PhaseShiftDecoder.h"

#include <cmath>
#include <vector>

namespace camcalib {
    namespace {

        constexpr double kTwoPi = 6.28318530717958647692;

        void validatePhaseImages(const std::vector<cv::Mat>& phaseImages) {
            CV_Assert(phaseImages.size() >= 3);
            CV_Assert(!phaseImages.front().empty());
            CV_Assert(phaseImages.front().type() == CV_32FC1);

            const cv::Size imageSize = phaseImages.front().size();
            for (const cv::Mat& image : phaseImages) {
                CV_Assert(!image.empty());
                CV_Assert(image.type() == CV_32FC1);
                CV_Assert(image.size() == imageSize);
            }
        }

    }  // namespace

    cv::Mat PhaseShiftDecoder::decode(const std::vector<cv::Mat>& phaseImages) {
        validatePhaseImages(phaseImages);

        const int phaseSteps = static_cast<int>(phaseImages.size());
        std::vector<float> cosWeights(phaseSteps);
        std::vector<float> sinWeights(phaseSteps);

        for (int step = 0; step < phaseSteps; ++step) {
            const double phaseShift =
                kTwoPi * static_cast<double>(step) /
                static_cast<double>(phaseSteps);
            cosWeights[step] = static_cast<float>(std::cos(phaseShift));
            sinWeights[step] = static_cast<float>(std::sin(phaseShift));
        }

        cv::Mat wrappedPhase(phaseImages.front().size(), CV_32FC1);
        std::vector<const float*> inputRows(phaseSteps);

        for (int row = 0; row < wrappedPhase.rows; ++row) {
            for (int step = 0; step < phaseSteps; ++step) {
                inputRows[step] = phaseImages[step].ptr<float>(row);
            }

            float* outputPtr = wrappedPhase.ptr<float>(row);
            for (int col = 0; col < wrappedPhase.cols; ++col) {
                float cosineComponent = 0.0f;
                float sineComponent = 0.0f;

                for (int step = 0; step < phaseSteps; ++step) {
                    const float intensity = inputRows[step][col];
                    cosineComponent += intensity * cosWeights[step];
                    sineComponent += intensity * sinWeights[step];
                }

                // 与旧四步实现保持相位符号一致：
                // N=4 时退化为 atan2(I3-I1, I0-I2)。
                outputPtr[col] = std::atan2(
                    -sineComponent,
                    cosineComponent
                );
            }
        }

        return wrappedPhase;
    }

}  // namespace camcalib
