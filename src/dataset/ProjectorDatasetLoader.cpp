#include "dataset/ProjectorDatasetLoader.h"

#include "utils/Config.h"
#include "utils/Logger.h"

#include <algorithm>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <utility>

namespace camcalib {
namespace {

constexpr size_t kImagesPerDirection = 12;

std::vector<std::filesystem::path> findPoseDirectories(
    const std::filesystem::path& root
){
    std::vector<std::filesystem::path> poseDirectories;
    if(!std::filesystem::exists(root) || !std::filesystem::is_directory(root)){
        return poseDirectories;
    }

    for(const std::filesystem::directory_entry& entry :
        std::filesystem::directory_iterator(root)){
        if(entry.is_directory()){
            poseDirectories.push_back(entry.path());
        }
    }

    std::sort(poseDirectories.begin(), poseDirectories.end());
    return poseDirectories;
}

bool loadImageGroup(
    const std::filesystem::path& directory,
    const std::vector<std::string>& extensions,
    std::vector<cv::Mat>& images
){
    const std::vector<std::string> imageFiles =
        ConfigReader::getImageFiles(directory.string(), extensions);
    if(imageFiles.size() != kImagesPerDirection){
        return false;
    }

    cv::Size expectedSize;
    images.clear();
    images.reserve(imageFiles.size());
    for(const std::string& imagePath : imageFiles){
        const cv::Mat grayImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        if(grayImage.empty()){
            return false;
        }
        if(expectedSize.empty()){
            expectedSize = grayImage.size();
        }else if(grayImage.size() != expectedSize){
            return false;
        }

        cv::Mat floatImage;
        grayImage.convertTo(floatImage, CV_32FC1);
        images.push_back(std::move(floatImage));
    }
    return true;
}

}  // namespace

ProjectorDatasetLoader::ProjectorDatasetLoader(
    ProjectorConfig projectorConfig,
    std::vector<std::string> imageExtensions
)
    : projectorConfig_(std::move(projectorConfig)),
      imageExtensions_(std::move(imageExtensions)) {}

std::vector<ProjectorPoseData> ProjectorDatasetLoader::load() const {
    std::vector<ProjectorPoseData> poses;
    const std::filesystem::path root =
        projectorConfig_.calibrationDataDirectory;

    for(const std::filesystem::path& poseDirectory : findPoseDirectories(root)){
        ProjectorPoseData pose;
        pose.poseName = poseDirectory.filename().string();

        const bool xLoaded = loadImageGroup(
            poseDirectory / "X",
            imageExtensions_,
            pose.xImages
        );
        const bool yLoaded = loadImageGroup(
            poseDirectory / "Y",
            imageExtensions_,
            pose.yImages
        );
        if(!xLoaded || !yLoaded ||
           pose.xImages.front().size() != pose.yImages.front().size()){
            utils::logError(
                "Skipped invalid projector pose directory: " +
                poseDirectory.string()
            );
            continue;
        }

        poses.push_back(std::move(pose));
    }

    return poses;
}

}  // namespace camcalib
