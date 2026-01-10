#pragma once

#include "common/track.h"
#include "odometry/landmark_odometry_solver.h"
#include "odometry/swerve_kinematics.h"
#include "simulation/synthetic_data.h"

#include <gtsam/geometry/Point3.h>
#include <vector>

struct SyntheticRunOutputs {
    Track2D ground_truth;
    Track2D vision_only;
    Track2D wheel_only;
    Track2D ekf;
    Track2D gtsam;
};

struct EstimatorScales {
    double gtsam_wheel_sigma_scale = 1.0;
    double gtsam_vision_sigma_scale = 1.0;
    double gtsam_gyro_sigma_scale = 1.0;
    double ekf_process_sigma_scale = 1.0;
    double ekf_measurement_sigma_scale = 1.0;
};

SyntheticRunOutputs runGtsamFromSynthetic(const SyntheticData& data,
                                          const SwerveParams& params,
                                          const std::vector<gtsam::Point3>& tag_positions,
                                          const MeasurementNoise& noise,
                                          const EstimatorScales& scales = EstimatorScales{});
