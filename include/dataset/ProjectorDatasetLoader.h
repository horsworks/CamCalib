#pragma once

#include "core/CalibrationTypes.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace camcalib {

/** @brief 单个投影仪标定位姿的原始图像、中间相位和最终特征点。 */
struct ProjectorPoseData {
    std::string poseName;                    ///< 位姿目录名称。
    std::vector<cv::Mat> xImages;            ///< 用于计算投影仪 X 坐标的12帧图像。
    std::vector<cv::Mat> yImages;            ///< 用于计算投影仪 Y 坐标的12帧图像。
    cv::Mat xAbsolutePhase;                  ///< X 方向绝对相位图，类型为 CV_32FC1。
    cv::Mat yAbsolutePhase;                  ///< Y 方向绝对相位图，类型为 CV_32FC1。
    std::vector<cv::Point2d> projectorPoints;  ///< 与相机特征点对应的投影仪像素坐标。
};

/** @brief 按位姿目录加载投影仪三频四步相移图像。 */
class ProjectorDatasetLoader {
public:
    /** @brief 创建投影仪数据加载器。
     *  @param projectorConfig 投影仪数据目录和标定配置。
     *  @param imageExtensions 允许读取的图像扩展名。
     */
    ProjectorDatasetLoader(
        ProjectorConfig projectorConfig,
        std::vector<std::string> imageExtensions
    );

    /** @brief 加载所有满足 X/Y 各12帧要求的位姿。
     *  @return 按位姿目录名称排序的数据。
     */
    std::vector<ProjectorPoseData> load() const;

private:
    ProjectorConfig projectorConfig_;           ///< 投影仪数据读取配置。
    std::vector<std::string> imageExtensions_;  ///< 允许读取的图像扩展名。
};

}  // namespace camcalib
