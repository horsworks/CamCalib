#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

class ReprojectionEvaluator {
public:
    EvaluationReport evaluate(const DetectionResult& detection, CalibrationResult& calibration) const;
};

}  // namespace camcalib
