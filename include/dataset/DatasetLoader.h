#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

class DatasetLoader {
public:
    explicit DatasetLoader(CalibrationPipelineConfig config);

    CalibrationDataset load() const;

private:
    CalibrationPipelineConfig config_;
};

}  // namespace camcalib
