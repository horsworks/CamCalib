#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

class DatasetLoader {
public:
    explicit DatasetLoader(DatasetConfig config);

    CalibrationDataset load() const;

private:
    DatasetConfig config_;
};

}  // namespace camcalib
