#include "pipeline/synthetic_pipeline.h"

#include "ekf/ekf_localization.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

std::vector<std::vector<LoopClosureMeasurement>> indexLoopClosures(
    size_t sample_count,
    const std::vector<LoopClosureMeasurement>& closures) {
    std::vector<std::vector<LoopClosureMeasurement>> out(sample_count);
    for (const auto& closure : closures) {
        if (closure.to_idx < out.size()) {
            out[closure.to_idx].push_back(closure);
        }
    }
    return out;
}

}

SyntheticRunOutputs runGtsamFromSynthetic(const SyntheticData& data,
                                          const SwerveParams& params,
                                          const std::vector<gtsam::Point3>& tag_positions,
                                          const MeasurementNoise& noise,
                                          const EstimatorScales& scales) {
    if (data.samples.size() < 2) {
        throw std::runtime_error("Not enough synthetic samples");
    }
    if (tag_positions.empty()) {
        throw std::runtime_error("No tag positions provided");
    }

    const gtsam::Pose3 initial_pose = data.ground_truth.p.front();
    MeasurementNoise gtsam_noise = noise;
    gtsam_noise.wheel_delta_sigma_m *= scales.gtsam_wheel_sigma_scale;
    gtsam_noise.wheel_angle_sigma_rad *= scales.gtsam_wheel_sigma_scale;
    gtsam_noise.vision_range_sigma_m *= scales.gtsam_vision_sigma_scale;
    gtsam_noise.vision_bearing_sigma_rad *= scales.gtsam_vision_sigma_scale;
    gtsam_noise.gyro_delta_sigma_rad *= scales.gtsam_gyro_sigma_scale;
    gtsam_noise.pitch_delta_sigma_rad *= scales.gtsam_gyro_sigma_scale;
    LandmarkOdometrySolver solver(initial_pose, tag_positions, gtsam_noise);

    gtsam::Matrix33 init_cov = gtsam::Matrix33::Identity() * 1e-4;
    EkfNoise ekf_noise;
    ekf_noise.process_sigma_x = noise.wheel_delta_sigma_m * scales.ekf_process_sigma_scale;
    ekf_noise.process_sigma_y = noise.wheel_delta_sigma_m * scales.ekf_process_sigma_scale;
    ekf_noise.process_sigma_theta =
        noise.wheel_angle_sigma_rad * scales.ekf_process_sigma_scale;
    ekf_noise.range_sigma =
        noise.vision_range_sigma_m * scales.ekf_measurement_sigma_scale;
    ekf_noise.bearing_sigma =
        noise.vision_bearing_sigma_rad * scales.ekf_measurement_sigma_scale;
    EkfLocalization ekf(gtsam::Pose2(0.0, 0.0, 0.0), init_cov);

    std::vector<gtsam::Point2> tag_positions_2d;
    tag_positions_2d.reserve(tag_positions.size());
    for (const auto& tag : tag_positions) {
        tag_positions_2d.emplace_back(tag.x(), tag.y());
    }

    Track2D vision_only;
    vision_only.t.reserve(data.samples.size() * tag_positions.size());
    vision_only.p.reserve(data.samples.size() * tag_positions.size());

    Track2D wheel_only;
    wheel_only.t.reserve(data.samples.size());
    wheel_only.p.reserve(data.samples.size());

    Track2D ekf_track;
    ekf_track.t.reserve(data.samples.size());
    ekf_track.p.reserve(data.samples.size());

    Track2D gtsam_track;
    gtsam_track.t.reserve(data.samples.size());
    gtsam_track.p.reserve(data.samples.size());

    const auto closures_by_to =
        indexLoopClosures(data.samples.size(), data.loop_closures);

    const double fixed_z = initial_pose.translation().z();
    gtsam::Pose2 wheel_pose(0.0, 0.0, 0.0);
    wheel_only.t.push_back(data.samples[0].t);
    wheel_only.p.push_back(
        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, wheel_pose.theta()),
                     gtsam::Point3(wheel_pose.x(), wheel_pose.y(), fixed_z)));
    ekf_track.t.push_back(data.samples[0].t);
    ekf_track.p.push_back(
        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, ekf.pose().theta()),
                     gtsam::Point3(ekf.pose().x(), ekf.pose().y(), fixed_z)));
    for (size_t i = 1; i < data.samples.size(); ++i) {
        const auto& prev = data.samples[i - 1];
        const auto& sample = data.samples[i];
        const double dt = sample.t - prev.t;
        if (dt <= 0.0) {
            continue;
        }

        const gtsam::Vector3 twist =
            solveTwistLeastSquaresFromDeltas(sample.wheel_delta_m, sample.wheel_angle_rad,
                                             params, dt);
        ekf.predict(twist(0), twist(1), twist(2), dt, ekf_noise);
        solver.addWheelOdometry(sample.wheel_delta_m,
                                sample.wheel_angle_rad,
                                dt,
                                params);
        const double gyro_sigma =
            noise.gyro_delta_sigma_rad *
            (1.0 + noise.gyro_drift_rate_per_s * sample.t);
        solver.addGyroDelta(sample.gyro_delta_rad, gyro_sigma);
        const double pitch_sigma =
            noise.pitch_delta_sigma_rad *
            (1.0 + noise.pitch_drift_rate_per_s * sample.t);
        solver.addPitchDelta(sample.pitch_delta_rad, pitch_sigma);
        for (const auto& measurement : sample.vision_measurements) {
            if (measurement.tag_index < tag_positions_2d.size()) {
                const auto& tag_pos = tag_positions_2d[measurement.tag_index];
                ekf.updateVision(tag_pos,
                                 measurement.distance_m,
                                 measurement.bearing_yaw_rad,
                                 ekf_noise);
            }
            solver.addVisionTagMeasurement(measurement.tag_index,
                                           measurement.distance_m,
                                           measurement.bearing_yaw_rad,
                                           measurement.bearing_pitch_rad);
        }
        ekf_track.t.push_back(sample.t);
        ekf_track.p.push_back(
            gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, ekf.pose().theta()),
                         gtsam::Point3(ekf.pose().x(), ekf.pose().y(), fixed_z)));

        const gtsam::Pose2 delta =
            deltaPoseFromWheelDeltas(sample.wheel_delta_m, sample.wheel_angle_rad, params, dt);
        wheel_pose = wheel_pose * delta;
        wheel_only.t.push_back(sample.t);
        wheel_only.p.push_back(
            gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, wheel_pose.theta()),
                         gtsam::Point3(wheel_pose.x(), wheel_pose.y(), fixed_z)));

        size_t closure_count = 0;
        if (i < closures_by_to.size()) {
            closure_count = closures_by_to[i].size();
            for (const auto& closure : closures_by_to[i]) {
                const gtsam::Key key1 = static_cast<gtsam::Key>(closure.from_idx + 1);
                const gtsam::Key key2 = static_cast<gtsam::Key>(closure.to_idx + 1);
                solver.addLoopClosure(key1, key2, closure.measured_delta);
            }
        }

        try {
            solver.update();
        } catch (const std::exception& e) {
            std::cerr << "iSAM2 update failed at step " << i
                      << " (t=" << sample.t << ", key=" << solver.getCurrentPoseKey()
                      << ", vision=" << sample.vision_measurements.size()
                      << ", loops=" << closure_count << ")\n";
            throw;
        }
    }

    const gtsam::Values final_estimate = solver.calculateEstimate();
    gtsam::Pose3 last_pose = solver.currentPoseEstimate();
    for (size_t i = 0; i < data.samples.size(); ++i) {
        const gtsam::Key key = static_cast<gtsam::Key>(i + 1);
        if (final_estimate.exists(key)) {
            last_pose = final_estimate.at<gtsam::Pose3>(key);
        }
        gtsam_track.t.push_back(data.samples[i].t);
        gtsam_track.p.push_back(last_pose);
    }

    const size_t vision_count =
        std::min(data.samples.size(), data.ground_truth.p.size());
    for (size_t i = 0; i < vision_count; ++i) {
        const auto& sample = data.samples[i];
        const auto& gt_pose = data.ground_truth.p[i];
        const double yaw = gt_pose.rotation().yaw();
        const double pitch = gt_pose.rotation().pitch();
        const gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(0.0, pitch, yaw);
        for (const auto& measurement : sample.vision_measurements) {
            if (measurement.tag_index >= tag_positions.size()) {
                continue;
            }
            const auto& tag_pos = tag_positions[measurement.tag_index];
            const double cos_pitch = std::cos(measurement.bearing_pitch_rad);
            const gtsam::Point3 rel_robot(
                measurement.distance_m * cos_pitch * std::cos(measurement.bearing_yaw_rad),
                measurement.distance_m * cos_pitch * std::sin(measurement.bearing_yaw_rad),
                measurement.distance_m * std::sin(measurement.bearing_pitch_rad));
            const gtsam::Point3 rel_world = rot.rotate(rel_robot);
            const double x = tag_pos.x() - rel_world.x();
            const double y = tag_pos.y() - rel_world.y();
            const double z = tag_pos.z() - rel_world.z();
            vision_only.t.push_back(sample.t);
            vision_only.p.emplace_back(rot, gtsam::Point3(x, y, z));
        }
    }

    SyntheticRunOutputs outputs;
    outputs.ground_truth = data.ground_truth;
    outputs.vision_only = std::move(vision_only);
    outputs.wheel_only = std::move(wheel_only);
    outputs.ekf = std::move(ekf_track);
    outputs.gtsam = std::move(gtsam_track);
    return outputs;
}
