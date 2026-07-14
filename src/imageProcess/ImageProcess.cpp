#include "imageProcess/ImageProcess.h"

#include "calibration/OpenCvCalibrator.h"
#include "dataset/DatasetLoader.h"
#include "detection/CircleGridDetector.h"
#include "evaluation/ReprojectionEvaluator.h"
#include "utils/Config.h"
#include "utils/Logger.h"
#include "utils/ResultIO.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace camcalib {

void ImageProcess::runCalibrate(){
    const std::string configPath = "config/calib_config.yaml";

    CaliConfig config;
    if(!ConfigReader::readConfig(configPath, config)){
        std::cerr << "Failed to read config file: " << configPath << std::endl;
        return;
    }

    utils::initializeLogger(config);
    utils::logInfo("Calibration started.");
    utils::logInfo("Current working directory: " + std::filesystem::current_path().string());
    utils::logInfo("Loaded config file: " + configPath);

    const CalibrationPipelineConfig pipelineConfig = ConfigReader::toPipelineConfig(config);
    const bool shouldSaveDebugImages = pipelineConfig.debug.saveImages;
    const std::filesystem::path debugRoot = pipelineConfig.debug.outputDirectory;

    if(shouldSaveDebugImages && !utils::prepareDebugOutputDirectory(debugRoot)){
        utils::logError("Failed to create debug output directory: " + debugRoot.string());
        utils::shutdownLogger();
        return;
    }

    DatasetLoader datasetLoader(pipelineConfig);
    const CalibrationDataset dataset = datasetLoader.load();
    if(dataset.empty()){
        utils::logError("Calibration aborted because no input images were loaded.");
        utils::shutdownLogger();
        return;
    }

    CircleGridDetector detector(pipelineConfig);
    DetectionResult detection = detector.detect(dataset);
    for(size_t imageIndex = 0; imageIndex < detection.views.size(); ++imageIndex){
        const ViewObservation& view = detection.views[imageIndex];
        if(view.valid){
            utils::logInfo(
                "Image " + std::to_string(imageIndex) +
                ": points=" + std::to_string(view.imagePoints.size()) +
                ", score=" + std::to_string(view.detectionScore)
            );
        }else{
            utils::logError(
                "Image " + std::to_string(imageIndex) +
                " detection failed: " + view.failureReason
            );
        }
    }

    OpenCvCalibrator calibrator(pipelineConfig.board);
    CalibrationResult calibration = calibrator.calibrate(dataset, detection);
    if(!calibration.converged){
        utils::logError("Calibration failed because no valid observations were produced.");
        utils::shutdownLogger();
        return;
    }
    utils::logInfo("Calibration finished. RMS = " + std::to_string(calibration.globalRmse));

    ReprojectionEvaluator evaluator;
    const EvaluationReport report = evaluator.evaluate(detection, calibration);
    for(const ViewError& viewError : report.views){
        std::ostringstream message;
        message << "Image " << viewError.viewIndex << " reprojection error = "
                << std::fixed << std::setprecision(6) << viewError.rmse
                << " pixels";
        utils::logInfo(message.str());
        std::cout << message.str() << std::endl;
    }

    utils::shutdownLogger();
}

}  // namespace camcalib
