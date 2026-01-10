#pragma once

#include <array>
#include <cstddef>
#include <unordered_set>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

struct SwerveParams;

struct MeasurementNoise {
    double wheel_delta_sigma_m = 0.02;
    double wheel_angle_sigma_rad = 0.01;
    double vision_range_sigma_m = 0.2;
    double vision_bearing_sigma_rad = 0.05;
    double loop_translation_sigma_m = 0.1;
    double loop_rotation_sigma_rad = 0.05;
    double tag_prior_sigma_m = 0.1;
    double gyro_delta_sigma_rad = 0.02;
    double gyro_drift_rate_per_s = 0.001;
    double pitch_delta_sigma_rad = 0.02;
    double pitch_drift_rate_per_s = 0.001;
};

class LandmarkOdometrySolver {
private:
    gtsam::ISAM2 isam_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
    gtsam::NonlinearFactorGraph full_graph_;
    gtsam::Values full_values_;
    gtsam::Values current_estimate_;
    gtsam::Key current_pose_key_ = 1;
    std::unordered_set<gtsam::Key> initialized_tags_;
    std::vector<gtsam::Point3> tag_positions_;

    gtsam::SharedNoiseModel vision_noise_;
    gtsam::SharedNoiseModel loop_closure_noise_;
    gtsam::SharedNoiseModel weak_pose_prior_;
    gtsam::SharedNoiseModel tag_prior_noise_;

    double wheel_delta_sigma_m_ = 0.0;
    double wheel_angle_sigma_rad_ = 0.0;

    gtsam::Pose3 poseGuessForCurrent() const;

public:
    explicit LandmarkOdometrySolver(const gtsam::Pose3& initial_pose,
                                    const std::vector<gtsam::Point3>& tag_positions,
                                    const MeasurementNoise& noise = MeasurementNoise{});

    void addWheelOdometry(const std::array<double, 4>& wheel_delta_m,
                          const std::array<double, 4>& wheel_angle_rad,
                          double dt,
                          const SwerveParams& params);
    void addVisionTagMeasurement(size_t tag_index,
                                 double distance_m,
                                 double bearing_yaw_rad,
                                 double bearing_pitch_rad);
    void addLoopClosure(gtsam::Key key1,
                        gtsam::Key key2,
                        const gtsam::Pose2& delta_pose);
    void addGyroDelta(double delta_theta_rad, double sigma_rad);
    void addPitchDelta(double delta_pitch_rad, double sigma_rad);
    void update();

    gtsam::Key getCurrentPoseKey() const;
    gtsam::Pose3 currentPoseEstimate() const;
    gtsam::Values calculateEstimate() const;
};
