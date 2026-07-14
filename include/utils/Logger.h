#pragma once

#include "utils/Config.h"
#include <string>

namespace camcalib::utils {

void initializeLogger(const CaliConfig& config);
void shutdownLogger();
void logInfo(const std::string& message);
void logError(const std::string& message);

}  // namespace camcalib::utils
