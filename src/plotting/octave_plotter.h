#pragma once

#include "common/track.h"

#include <gtsam/geometry/Point3.h>

#include <string>
#include <vector>

struct PlotOutputs {
    std::string ground_truth_csv;
    std::string wheel_only_csv;
    std::string vision_csv;
    std::string ekf_csv;
    std::string tag_csv;
    std::string fused_csv;
    std::string plot_png;
    std::string plot_with_raw_png;
    std::string yaw_plot_png;
};

PlotOutputs writePlotInputs(const Track2D& ground_truth,
                            const Track2D& wheel_only,
                            const Track2D& vision_only,
                            const Track2D& ekf_track,
                            const Track2D& fused,
                            const std::vector<gtsam::Point3>& tag_positions,
                            const std::string& output_dir);

bool runOctavePlot(const PlotOutputs& outputs, const std::string& script_path);
