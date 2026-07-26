#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

/** @brief 检测圆点标定板并生成排序后的标定观测。 */
class CircleGridDetector {
public:
    /** @brief 创建圆点标定板检测器。
     *  @param boardConfig 标定板行列及间距配置。
     *  @param detectorConfig 轮廓筛选、Marker 和亚像素配置。
     */
    CircleGridDetector(BoardConfig boardConfig, DetectorConfig detectorConfig);

    /** @brief 检测数据集中的所有圆点标定板。
     *  @param dataset 输入图像数据集。
     *  @return 各检测阶段结果和最终世界点/图像点观测。
     */
    DetectionResult detect(const CalibrationDataset& dataset) const;

private:
    BoardConfig boardConfig_;        ///< 标定板几何配置。
    DetectorConfig detectorConfig_;  ///< 圆点检测配置。
};

}  // namespace camcalib
