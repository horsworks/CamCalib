#include "dataset/DatasetLoader.h"

#include "utils/Config.h"
#include "utils/Logger.h"

#include <opencv2/imgcodecs.hpp>
#include <utility>

namespace camcalib {

DatasetLoader::DatasetLoader(DatasetConfig config)
    : config_(std::move(config)) {}

CalibrationDataset DatasetLoader::load() const {
    CalibrationDataset dataset;

    const std::vector<std::string> imageFiles = ConfigReader::getImageFiles(
        config_.imageDirectory,
        config_.imageExtensions
    );
    if(imageFiles.empty()){
        utils::logError("No image files found in directory: " + config_.imageDirectory);
        return dataset;
    }

    utils::logInfo(
        "Found " + std::to_string(imageFiles.size()) +
        " image files in " + config_.imageDirectory
    );

    for(const std::string& path : imageFiles){
        const int readFlag = config_.readGrayscale ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR;
        cv::Mat image = cv::imread(path, readFlag);
        if(image.empty()){
            utils::logError("Failed to load image: " + path);
            continue;
        }

        if(dataset.images.empty()){
            dataset.imageSize = image.size();
        }else if(image.size() != dataset.imageSize){
            utils::logError(
                "Skipping image with inconsistent size: " + path +
                " expected=" + std::to_string(dataset.imageSize.width) + "x" +
                std::to_string(dataset.imageSize.height) +
                " actual=" + std::to_string(image.cols) + "x" + std::to_string(image.rows)
            );
            continue;
        }

        dataset.images.push_back({path, image});
    }

    return dataset;
}

}  // namespace camcalib
