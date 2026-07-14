#include "utils/Config.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string_view>

namespace fs = std::filesystem;

namespace camcalib {

bool ConfigReader::readConfig(const std::string& yaml_path, CaliConfig& config) {
    cv::FileStorage fileStorage(yaml_path, cv::FileStorage::READ);
    if (!fileStorage.isOpened()) {
        std::cerr << "Failed to open config file: " << yaml_path << std::endl;
        return false;
    }

    fileStorage["image_dir"] >> config.image_dir;

    config.image_extensions.clear();
    cv::FileNode extNode = fileStorage["image_extensions"];
    if (extNode.type() == cv::FileNode::SEQ) {
        for (const auto& ext : extNode) {
            config.image_extensions.push_back(static_cast<std::string>(ext));
        }
    }

    int readGrayScale = config.read_grayscale ? 1 : 0;
    if (!fileStorage["read_grayscale"].empty()) {
        fileStorage["read_grayscale"] >> readGrayScale;
    }
    config.read_grayscale = (readGrayScale != 0);

    fileStorage["calib_rows"] >> config.calib_rows;
    fileStorage["calib_cols"] >> config.calib_cols;
    fileStorage["calib_centerDistance"] >> config.calib_centerDistance;

    if (!fileStorage["detector_min_contour_points"].empty()) {
        fileStorage["detector_min_contour_points"] >> config.detector_min_contour_points;
    }
    if (!fileStorage["detector_max_axis_ratio"].empty()) {
        fileStorage["detector_max_axis_ratio"] >> config.detector_max_axis_ratio;
    }
    if (!fileStorage["detector_marker_count"].empty()) {
        fileStorage["detector_marker_count"] >> config.detector_marker_count;
    }
    if (!fileStorage["detector_marker_spacing"].empty()) {
        fileStorage["detector_marker_spacing"] >> config.detector_marker_spacing;
    }
    if (!fileStorage["detector_row_tolerance"].empty()) {
        fileStorage["detector_row_tolerance"] >> config.detector_row_tolerance;
    }
    int enableSubpixel = config.detector_enable_subpixel ? 1 : 0;
    if (!fileStorage["detector_enable_subpixel"].empty()) {
        fileStorage["detector_enable_subpixel"] >> enableSubpixel;
    }
    config.detector_enable_subpixel = (enableSubpixel != 0);

    int logEnabled = config.log_enabled ? 1 : 0;
    if (!fileStorage["log_enabled"].empty()) {
        fileStorage["log_enabled"] >> logEnabled;
    }
    config.log_enabled = (logEnabled != 0);

    if (!fileStorage["log_output_file"].empty()) {
        fileStorage["log_output_file"] >> config.log_output_file;
    }

    int debugMode = config.debug_mode ? 1 : 0;
    if (!fileStorage["debug_mode"].empty()) {
        fileStorage["debug_mode"] >> debugMode;
    }
    config.debug_mode = (debugMode != 0);

    int debugSaveImages = config.debug_save_images ? 1 : 0;
    if (!fileStorage["debug_save_images"].empty()) {
        fileStorage["debug_save_images"] >> debugSaveImages;
    }
    config.debug_save_images = (debugSaveImages != 0);

    int debugShowWindows = config.debug_show_windows ? 1 : 0;
    if (!fileStorage["debug_show_windows"].empty()) {
        fileStorage["debug_show_windows"] >> debugShowWindows;
    }
    config.debug_show_windows = (debugShowWindows != 0);

    if (!fileStorage["debug_output_dir"].empty()) {
        fileStorage["debug_output_dir"] >> config.debug_output_dir;
    }

    fileStorage.release();
    return true;
}

CalibrationPipelineConfig ConfigReader::toPipelineConfig(const CaliConfig& config) {
    CalibrationPipelineConfig pipelineConfig;
    pipelineConfig.imageDirectory = config.image_dir;
    pipelineConfig.imageExtensions = config.image_extensions;
    pipelineConfig.readGrayscale = config.read_grayscale;

    pipelineConfig.board.rows = config.calib_rows;
    pipelineConfig.board.cols = config.calib_cols;
    pipelineConfig.board.spacingMm = config.calib_centerDistance;

    pipelineConfig.detector.minContourPoints = config.detector_min_contour_points;
    pipelineConfig.detector.maxAxisRatio = config.detector_max_axis_ratio;
    pipelineConfig.detector.markerCount = config.detector_marker_count;
    pipelineConfig.detector.markerSpacing = config.detector_marker_spacing;
    pipelineConfig.detector.rowTolerance = config.detector_row_tolerance;
    pipelineConfig.detector.enableSubpixel = config.detector_enable_subpixel;

    pipelineConfig.logEnabled = config.log_enabled;
    pipelineConfig.logOutputFile = config.log_output_file;
    pipelineConfig.debugMode = config.debug_mode;
    pipelineConfig.debug.saveImages = config.debug_save_images;
    pipelineConfig.debug.showWindows = config.debug_show_windows;
    pipelineConfig.debug.outputDirectory = config.debug_output_dir;
    return pipelineConfig;
}

std::vector<std::string> ConfigReader::getImageFiles(
    const std::string& dir,
    const std::vector<std::string>& extensions
) {
    std::vector<std::string> files;

    std::vector<std::string> normalizedExtensions = extensions;
    for (auto& ext : normalizedExtensions) {
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
    }

    if (normalizedExtensions.empty()) {
        normalizedExtensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"};
    }

    try {
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) {
                    return static_cast<char>(std::tolower(ch));
                });

                if (std::find(normalizedExtensions.begin(), normalizedExtensions.end(), ext) != normalizedExtensions.end()) {
                    files.push_back(entry.path().string());
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
    }

    auto naturalCompare = [](const std::string& a, const std::string& b) {
        size_t i = 0, j = 0;
        while (i < a.size() && j < b.size()) {
            if (std::isdigit(static_cast<unsigned char>(a[i])) && std::isdigit(static_cast<unsigned char>(b[j]))) {
                size_t i0 = i, j0 = j;
                while (i0 < a.size() && std::isdigit(static_cast<unsigned char>(a[i0]))) ++i0;
                while (j0 < b.size() && std::isdigit(static_cast<unsigned char>(b[j0]))) ++j0;

                std::string_view sa(a.c_str() + i, i0 - i);
                std::string_view sb(b.c_str() + j, j0 - j);

                // skip leading zeros for numeric comparison
                size_t k = 0;
                while (k < sa.size() && sa[k] == '0') ++k;
                size_t l = 0;
                while (l < sb.size() && sb[l] == '0') ++l;
                std::string_view sa_trim = sa.substr(k);
                std::string_view sb_trim = sb.substr(l);

                if (sa_trim.size() != sb_trim.size())
                    return sa_trim.size() < sb_trim.size();
                if (sa_trim != sb_trim)
                    return sa_trim < sb_trim;
                if (sa.size() != sb.size())
                    return sa.size() < sb.size();

                i = i0;
                j = j0;
                continue;
            }
            if (a[i] != b[j])
                return a[i] < b[j];
            ++i;
            ++j;
        }
        return a.size() < b.size();
    };

    std::sort(files.begin(), files.end(), naturalCompare);
    return files;
}

}  // namespace camcalib
