#pragma once

#include "core/CalibrationTypes.h"

#include <string>
#include <vector>

namespace camcalib {

struct CaliConfig {
    std::string image_dir;
    std::vector<std::string> image_extensions;
    bool read_grayscale = true;

    int calib_rows = 0;
    int calib_cols = 0;
    double calib_centerDistance = 0.0;

    int detector_min_contour_points = 20;
    double detector_max_axis_ratio = 1.5;
    int detector_marker_count = 5;
    double detector_marker_spacing = 150.0;
    double detector_row_tolerance = 7.5;
    bool detector_enable_subpixel = true;

    bool log_enabled = true;
    std::string log_output_file = "debug_output/run.log";

    bool debug_mode = false;
    bool debug_save_images = true;
    bool debug_show_windows = false;
    std::string debug_output_dir = "debug_output";
};

class ConfigReader {
public:
    static bool readConfig(const std::string& yaml_path, CaliConfig& config);
    static CalibrationPipelineConfig toPipelineConfig(const CaliConfig& config);

    static std::vector<std::string> getImageFiles(
        const std::string& dir,
        const std::vector<std::string>& extensions
    );
};

}
