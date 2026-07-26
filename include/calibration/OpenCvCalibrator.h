#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

/** @brief 使用 OpenCV 标定针孔相机或伪相机模型设备。 */
class OpenCvCalibrator {
public:
    /** @brief 创建标定器。
     *  @param boardConfig 标定板几何配置。
     */
    explicit OpenCvCalibrator(BoardConfig boardConfig);

    /** @brief 根据多位姿世界点与像素点计算内参、畸变和外参。
     *  @param dataset 提供设备图像尺寸的标定数据集。
     *  @param detection 每个位姿的世界点和像素点观测。
     *  @return 标定参数及全局重投影误差。
     */
    CalibrationResult calibrate(const CalibrationDataset& dataset, const DetectionResult& detection) const;
};

}  // namespace camcalib
