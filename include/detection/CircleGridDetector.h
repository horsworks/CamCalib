#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

class CircleGridDetector {
public:
    CircleGridDetector(BoardConfig boardConfig, DetectorConfig detectorConfig);

    DetectionResult detect(const CalibrationDataset& dataset) const;

private:
    BoardConfig boardConfig_;
    DetectorConfig detectorConfig_;
};

}  // namespace camcalib
