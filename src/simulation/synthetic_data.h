#pragma once

#include "common/track.h"
#include "odometry/swerve_kinematics.h"

#include <array>
#include <cstddef>
#include <random>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>

struct NoiseSpec {
    double mean = 0.0;
    double stddev = 0.0;
};

enum class TrajectoryProfile {
    Simple,
    Complex
};

struct SensorNoiseConfig {
    NoiseSpec wheel_delta_m;
    NoiseSpec wheel_angle_rad;
    NoiseSpec vision_range_m;
    NoiseSpec vision_pixel_px;
    NoiseSpec loop_translation_m;
    NoiseSpec loop_rotation_rad;
    NoiseSpec gyro_delta_rad;
    NoiseSpec gyro_drift_rate_per_s;
    NoiseSpec pitch_delta_rad;
    NoiseSpec pitch_drift_rate_per_s;
};

struct VisionCameraModel {
    int image_width_px = 1280;
    double fov_deg = 70.0;
    double principal_x_px = 640.0;
};

struct SyntheticConfig {
    double duration_s = 5.0;
    double sample_rate_hz = 50.0;
    std::vector<gtsam::Point3> tag_positions{gtsam::Point3(2.0, 0.0, 1.0)};
    double robot_z_m = 0.0;
    SensorNoiseConfig noise;
    VisionCameraModel camera;
    TrajectoryProfile trajectory = TrajectoryProfile::Simple;
    unsigned int seed = 42;
};

struct VisionMeasurement {
    size_t tag_index = 0;
    double distance_m = 0.0;
    double bearing_yaw_rad = 0.0;
    double bearing_pitch_rad = 0.0;
};

struct SyntheticSample {
    double t = 0.0;
    std::array<double, 4> wheel_delta_m{};
    std::array<double, 4> wheel_angle_rad{};
    double gyro_delta_rad = 0.0;
    double pitch_delta_rad = 0.0;
    std::vector<VisionMeasurement> vision_measurements;
};

struct LoopClosureMeasurement {
    size_t from_idx = 0;
    size_t to_idx = 0;
    gtsam::Pose2 measured_delta;
};

struct NoiseStats {
    double mean = 0.0;
    double variance = 0.0;
};

struct SensorNoiseStats {
    NoiseStats wheel_delta_m;
    NoiseStats wheel_angle_rad;
    NoiseStats vision_range_m;
    NoiseStats vision_bearing_rad;
    NoiseStats vision_pixel_px;
    NoiseStats loop_translation_m;
    NoiseStats loop_rotation_rad;
    NoiseStats gyro_delta_rad;
    NoiseStats pitch_delta_rad;
};

struct MeasurementSigmas {
    double wheel_delta_m = 0.0;
    double wheel_angle_rad = 0.0;
    double vision_range_m = 0.0;
    double vision_bearing_rad = 0.0;
    double loop_translation_m = 0.0;
    double loop_rotation_rad = 0.0;
    double gyro_delta_rad = 0.0;
    double pitch_delta_rad = 0.0;
};

struct SyntheticData {
    Track2D ground_truth;
    std::vector<SyntheticSample> samples;
    std::vector<LoopClosureMeasurement> loop_closures;
    SensorNoiseStats noise_stats;
    MeasurementSigmas sigmas;
};

SyntheticData generateSyntheticData(const SyntheticConfig& cfg, const SwerveParams& params);
