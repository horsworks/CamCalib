#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"

#include <vector>

namespace camcalib {

/** @brief 在绝对相位图中采样相机特征点并计算投影仪像素坐标。 */
class ProjectorPointMatcher {
public:
    /** @brief 为每个位姿填充投影仪特征点。
     *  @param cameraViews 相机检测得到的世界点和圆心坐标。
     *  @param poses 包含绝对相位图的投影仪位姿，结果写入 projectorPoints。
     *  @param projectorConfig 投影仪分辨率和最高相位频率配置。
     */
    void match(
        const std::vector<ViewObservation>& cameraViews,
        std::vector<ProjectorPoseData>& poses,
        const ProjectorConfig& projectorConfig
    ) const;
};

}  // namespace camcalib
