#pragma once

#include "core/CalibrationTypes.h"
#include <string>

namespace camcalib::utils {

void initializeLogger(const LoggingConfig& config);
void shutdownLogger();
void logInfo(const std::string& message);
void logError(const std::string& message);

}  // namespace camcalib::utils
