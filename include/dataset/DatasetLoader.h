#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

/** @brief 从配置目录加载尺寸一致的相机标定图像。 */
class DatasetLoader {
public:
    /** @brief 创建相机数据集加载器。
     *  @param config 图像目录、扩展名和读取模式配置。
     */
    explicit DatasetLoader(DatasetConfig config);

    /** @brief 加载并校验相机标定图像。
     *  @return 按自然顺序排列的标定数据集。
     */
    CalibrationDataset load() const;

private:
    DatasetConfig config_;  ///< 相机数据集读取配置。
};

}  // namespace camcalib
