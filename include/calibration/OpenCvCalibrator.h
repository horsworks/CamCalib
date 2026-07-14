#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

class OpenCvCalibrator {
public:
    explicit OpenCvCalibrator(BoardConfig boardConfig);

    CalibrationResult calibrate(const CalibrationDataset& dataset, const DetectionResult& detection) const;

private:
    BoardConfig boardConfig_;
};

}  // namespace camcalib
