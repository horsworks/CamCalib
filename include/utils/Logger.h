#pragma once

#include "core/CalibrationTypes.h"
#include <string>

namespace camcalib::utils {

/** @brief 根据配置初始化全局日志输出。 */
void initializeLogger(const LoggingConfig& config);

/** @brief 刷新并关闭全局日志文件。 */
void shutdownLogger();

/** @brief 写入一条信息日志。
 *  @param message 日志正文。
 */
void logInfo(const std::string& message);

/** @brief 写入一条错误日志。
 *  @param message 错误正文。
 */
void logError(const std::string& message);

}  // namespace camcalib::utils
