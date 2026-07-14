#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

class CircleGridDetector {
public:
    explicit CircleGridDetector(CalibrationPipelineConfig config);

    DetectionResult detect(const CalibrationDataset& dataset) const;

private:
    CalibrationPipelineConfig config_;
};

}  // namespace camcalib
