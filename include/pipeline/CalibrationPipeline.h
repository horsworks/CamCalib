#pragma once

#include "core/CalibrationTypes.h"
#include "dataset/ProjectorDatasetLoader.h"

#include <array>
#include <string>
#include <vector>

namespace camcalib {

/** @brief 编排相机检测、相机标定和伪相机法投影仪标定流程。 */
class CalibrationPipeline {
public:
    /** @brief 执行相机数据加载、圆点检测、标定和评价。
     *  @param configPath YAML 配置文件路径。
     *  @param cameraDetection 输出供投影仪流程复用的相机圆心和世界点。
     *  @return 相机标定是否成功。
     */
    bool runCameraCalibration(
        const std::string& configPath,
        DetectionResult& cameraDetection
    ) const;

    /** @brief 执行投影仪数据加载、相位解算、点匹配、标定和评价。
     *  @param configPath YAML 配置文件路径。
     *  @param cameraViews 相机流程生成的各位姿圆心和世界点。
     */
    void runProjectorCalibration(
        const std::string& configPath,
        const std::vector<ViewObservation>& cameraViews
    ) const;

private:
    /** @brief 组装投影仪标定器和评价器需要的观测结果。 */
    DetectionResult buildProjectorDetection(
        const std::vector<ViewObservation>& cameraViews,
        const std::vector<ProjectorPoseData>& poses,
        cv::Size projectorSize
    ) const;

    /** @brief 为所有投影仪位姿计算 X/Y 绝对相位。 */
    bool solveProjectorPhases(
        std::vector<ProjectorPoseData>& poses,
        const std::array<float, 3>& frequencies,
        int minValidViews
    ) const;

    /** @brief 执行相机或投影仪共用的标定和重投影评价。 */
    CalibrationResult calibrateAndEvaluate(
        const CalibrationDataset& dataset,
        DetectionResult& detection,
        const BoardConfig& board,
        EvaluationReport& evaluation
    ) const;
};

}  // namespace camcalib
