#pragma once

#include "core/CalibrationTypes.h"

namespace camcalib {

/** @brief 计算标定结果的逐位姿重投影误差。 */
class ReprojectionEvaluator {
public:
    /** @brief 评价有效观测的重投影误差。
     *  @param detection 标定时使用的世界点和像素点观测。
     *  @param calibration 已完成的标定结果。
     *  @return 全局、平均、最大和逐位姿误差报告。
     */
    EvaluationReport evaluate(const DetectionResult& detection, CalibrationResult& calibration) const;
};

}  // namespace camcalib
