#pragma once

#include "core/CalibrationTypes.h"

#include <string>
#include <vector>

namespace camcalib {

/** @brief OpenCV YAML 配置和图像文件列表读取工具。 */
class ConfigReader {
public:
    /** @brief 读取并校验标定 Pipeline 配置。
     *  @param yamlPath YAML 配置文件路径。
     *  @param config 输出配置对象。
     *  @return 文件可读且全部配置有效时返回 true。
     */
    static bool readConfig(const std::string& yamlPath, CalibrationPipelineConfig& config);

    /** @brief 获取目录中指定扩展名的图像文件。
     *  @param dir 待扫描目录。
     *  @param extensions 允许的扩展名列表。
     *  @return 按自然文件名顺序排列的完整路径。
     */
    static std::vector<std::string> getImageFiles(
        const std::string& dir,
        const std::vector<std::string>& extensions
    );
};

}  // namespace camcalib
