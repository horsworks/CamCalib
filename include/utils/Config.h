#pragma once

#include "core/CalibrationTypes.h"

#include <string>
#include <vector>

namespace camcalib {

class ConfigReader {
public:
    static bool readConfig(const std::string& yamlPath, CalibrationPipelineConfig& config);

    static std::vector<std::string> getImageFiles(
        const std::string& dir,
        const std::vector<std::string>& extensions
    );
};

}
