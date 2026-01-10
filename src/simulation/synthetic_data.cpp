#include "simulation/synthetic_data.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

double normalizeAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

struct NoiseAccumulator {
    double sum = 0.0;
    double sum_sq = 0.0;
    size_t count = 0;

    void add(double v) {
        sum += v;
        sum_sq += v * v;
        ++count;
    }

    NoiseStats stats() const {
        NoiseStats out;
        if (count == 0) return out;
        out.mean = sum / static_cast<double>(count);
        const double mean_sq = out.mean * out.mean;
        const double avg_sq = sum_sq / static_cast<double>(count);
        out.variance = std::max(0.0, avg_sq - mean_sq);
        return out;
    }
};

struct Twist {
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
};

Twist commandedTwistSimple(double t) {
    Twist out;
    out.vx = 1.0 + 0.2 * std::sin(2.0 * kPi * 0.2 * t);
    out.vy = 0.4 * std::sin(2.0 * kPi * 0.1 * t);
    out.omega = 0.3 * std::cos(2.0 * kPi * 0.15 * t);
    return out;
}

Twist commandedTwistComplex(double t, double theta) {
    Twist out;
    const double a1 = 2.0;
    const double a2 = 1.0;
    const double b1 = 1.6;
    const double b2 = 0.7;
    const double f1 = 0.05;
    const double f2 = 0.12;
    const double f3 = 0.07;
    const double f4 = 0.17;

    const double vx_world =
        2.0 * kPi * (a1 * f1 * std::cos(2.0 * kPi * f1 * t) +
                     a2 * f2 * std::cos(2.0 * kPi * f2 * t + 0.7));
    const double vy_world =
        2.0 * kPi * (b1 * f3 * std::cos(2.0 * kPi * f3 * t + 1.1) +
                     b2 * f4 * std::cos(2.0 * kPi * f4 * t + 0.3));

    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    out.vx = cos_t * vx_world + sin_t * vy_world;
    out.vy = -sin_t * vx_world + cos_t * vy_world;
    out.omega = 0.4 * std::sin(2.0 * kPi * 0.06 * t)
              + 0.25 * std::sin(2.0 * kPi * 0.11 * t + 0.4);

    const double phase = std::fmod(t, 10.0);
    double scale = 1.0;
    if (phase < 1.5) {
        scale = phase / 1.5;
    } else if (phase > 8.5) {
        scale = (10.0 - phase) / 1.5;
    }
    out.vx *= scale;
    out.vy *= scale;
    out.omega *= scale;
    return out;
}

Twist selectTwist(const SyntheticConfig& cfg, double t, double theta) {
    switch (cfg.trajectory) {
        case TrajectoryProfile::Complex:
            return commandedTwistComplex(t, theta);
        case TrajectoryProfile::Simple:
        default:
            return commandedTwistSimple(t);
    }
}

double pitchProfile(double) {
    return 0.0;
}

double wheelNoiseScale(const Twist& twist) {
    const double v = std::hypot(twist.vx, twist.vy);
    const double omega = std::abs(twist.omega);
    const double v_thresh = 0.2;
    const double omega_thresh = 0.15;
    if (v < v_thresh || omega < omega_thresh) {
        return 1.0;
    }
    const double v_ref = 1.0;
    const double omega_ref = 0.8;
    const double mix_gain = 0.3;
    const double curvature = omega / std::max(v, 1e-3);
    const double curvature_ref = 0.5;
    const double curve_gain = 0.4;
    const double mix = (v / v_ref) * (omega / omega_ref);
    const double curve_term = std::max(0.0, (curvature - curvature_ref) / curvature_ref);
    double scale = 1.0 + mix_gain * mix + curve_gain * curve_term;
    const double max_scale = 1.8;
    if (scale > max_scale) {
        scale = max_scale;
    }
    return scale;
}

}

SyntheticData generateSyntheticData(const SyntheticConfig& cfg, const SwerveParams& params) {
    SyntheticData out;
    const double dt = (cfg.sample_rate_hz > 0.0) ? (1.0 / cfg.sample_rate_hz) : 0.0;
    const int sample_count = static_cast<int>(cfg.duration_s * cfg.sample_rate_hz);
    if (sample_count < 2 || dt <= 0.0) {
        return out;
    }

    std::mt19937 rng(cfg.seed);
    std::normal_distribution<double> normal(0.0, 1.0);

    auto sample_noise = [&](const NoiseSpec& spec) {
        return spec.mean + spec.stddev * normal(rng);
    };

    const double fov_rad = degToRad(cfg.camera.fov_deg);
    const double half_fov_rad = 0.5 * fov_rad;
    const double focal_length_px =
        (static_cast<double>(cfg.camera.image_width_px) * 0.5) / std::tan(0.5 * fov_rad);

    std::vector<gtsam::Point3> tags = cfg.tag_positions;
    if (tags.empty()) {
        tags.emplace_back(2.0, 0.0, 1.0);
    }

    NoiseAccumulator wheel_delta_stats;
    NoiseAccumulator wheel_angle_stats;
    NoiseAccumulator vision_range_stats;
    NoiseAccumulator vision_bearing_stats;
    NoiseAccumulator vision_pixel_stats;
    NoiseAccumulator loop_translation_stats;
    NoiseAccumulator loop_rotation_stats;
    NoiseAccumulator gyro_delta_stats;
    NoiseAccumulator pitch_delta_stats;

    out.samples.resize(static_cast<size_t>(sample_count));
    out.ground_truth.t.reserve(static_cast<size_t>(sample_count));
    out.ground_truth.p.reserve(static_cast<size_t>(sample_count));

    gtsam::Pose2 pose2(0.0, 0.0, 0.0);
    double pitch = pitchProfile(0.0);
    double prev_pitch = pitch;
    gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.0, pitch, pose2.theta()),
                      gtsam::Point3(pose2.x(), pose2.y(), cfg.robot_z_m));
    out.ground_truth.t.push_back(0.0);
    out.ground_truth.p.push_back(pose);

    const double min_range_m = 0.25;

    auto addVisionMeasurements = [&](SyntheticSample& sample, const gtsam::Pose3& pose) {
        sample.vision_measurements.clear();
        sample.vision_measurements.reserve(tags.size());
        for (size_t tag_idx = 0; tag_idx < tags.size(); ++tag_idx) {
            const auto& tag = tags[tag_idx];
            const gtsam::Point3 rel = pose.transformTo(tag);
            const double dx = rel.x();
            const double dy = rel.y();
            const double dz = rel.z();
            const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (range < min_range_m) {
                continue;
            }
            if (dx <= 0.0) {
                continue;
            }
            const double azimuth = std::atan2(dy, dx);
            const double horiz = std::hypot(dx, dy);
            const double elevation = std::atan2(dz, horiz);
            if (std::abs(azimuth) > half_fov_rad ||
                std::abs(elevation) > half_fov_rad) {
                continue;
            }

            const double range_scale = std::min(1.0 + 0.1 * range, 3.0);
            const double pixel_noise_yaw =
                sample_noise(cfg.noise.vision_pixel_px) * range_scale;
            const double pixel_noise_pitch =
                sample_noise(cfg.noise.vision_pixel_px) * range_scale;
            double bearing_yaw_meas = azimuth;
            double bearing_pitch_meas = elevation;
            if (std::abs(focal_length_px) > 1e-9) {
                const double pixel_x = focal_length_px * std::tan(azimuth);
                const double pixel_y = focal_length_px * std::tan(elevation);
                const double pixel_x_meas = pixel_x + pixel_noise_yaw;
                const double pixel_y_meas = pixel_y + pixel_noise_pitch;
                bearing_yaw_meas = std::atan(pixel_x_meas / focal_length_px);
                bearing_pitch_meas = std::atan(pixel_y_meas / focal_length_px);

                const double yaw_wrap = std::round((azimuth - bearing_yaw_meas) / kPi);
                bearing_yaw_meas = normalizeAngle(bearing_yaw_meas + yaw_wrap * kPi);
                const double pitch_wrap = std::round((elevation - bearing_pitch_meas) / kPi);
                bearing_pitch_meas = normalizeAngle(bearing_pitch_meas + pitch_wrap * kPi);
            } else {
                bearing_yaw_meas = normalizeAngle(bearing_yaw_meas);
                bearing_pitch_meas = normalizeAngle(bearing_pitch_meas);
            }
            const double range_meas = range + sample_noise(cfg.noise.vision_range_m);

            sample.vision_measurements.push_back(
                VisionMeasurement{tag_idx, range_meas, bearing_yaw_meas, bearing_pitch_meas});

            vision_range_stats.add(range_meas - range);
            vision_bearing_stats.add(normalizeAngle(bearing_yaw_meas - azimuth));
            vision_bearing_stats.add(normalizeAngle(bearing_pitch_meas - elevation));
            vision_pixel_stats.add(pixel_noise_yaw);
            vision_pixel_stats.add(pixel_noise_pitch);
        }
    };

    SyntheticSample first_sample;
    first_sample.t = 0.0;
    addVisionMeasurements(first_sample, pose);
    out.samples[0] = std::move(first_sample);

    for (int i = 1; i < sample_count; ++i) {
        const double t_prev = static_cast<double>(i - 1) * dt;
        const double t = static_cast<double>(i) * dt;
        const double t_mid = t_prev + 0.5 * dt;
        const Twist twist = selectTwist(cfg, t_mid, pose2.theta());

        const gtsam::Pose2 delta(twist.vx * dt, twist.vy * dt, twist.omega * dt);
        pose2 = pose2 * delta;
        pitch = pitchProfile(t);
        const double pitch_delta = normalizeAngle(pitch - prev_pitch);
        prev_pitch = pitch;
        pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, pitch, pose2.theta()),
                            gtsam::Point3(pose2.x(), pose2.y(), cfg.robot_z_m));

        out.ground_truth.t.push_back(t);
        out.ground_truth.p.push_back(pose);

        SyntheticSample sample;
        sample.t = t;
        const double wheel_noise_scale = wheelNoiseScale(twist);
        NoiseSpec wheel_delta_spec = cfg.noise.wheel_delta_m;
        NoiseSpec wheel_angle_spec = cfg.noise.wheel_angle_rad;
        wheel_delta_spec.stddev *= wheel_noise_scale;
        wheel_angle_spec.stddev *= wheel_noise_scale;

        for (int w = 0; w < 4; ++w) {
            const size_t idx = static_cast<size_t>(w);
            const auto [rx, ry] = params.module_pos[idx];
            const double vix = twist.vx - twist.omega * ry;
            const double viy = twist.vy + twist.omega * rx;
            const double speed = std::hypot(vix, viy);
            const double angle = std::atan2(viy, vix);
            const double delta_dist = speed * dt;

            const double noisy_delta = delta_dist + sample_noise(wheel_delta_spec);
            const double noisy_angle =
                normalizeAngle(angle + sample_noise(wheel_angle_spec));

            sample.wheel_delta_m[idx] = noisy_delta;
            sample.wheel_angle_rad[idx] = noisy_angle;

            wheel_delta_stats.add(noisy_delta - delta_dist);
            wheel_angle_stats.add(normalizeAngle(noisy_angle - angle));
        }

        NoiseSpec gyro_spec = cfg.noise.gyro_delta_rad;
        const double gyro_scale = 1.0 + cfg.noise.gyro_drift_rate_per_s.stddev * t;
        gyro_spec.stddev *= gyro_scale;
        const double gyro_meas = delta.theta() + sample_noise(gyro_spec);
        sample.gyro_delta_rad = gyro_meas;
        gyro_delta_stats.add(gyro_meas - delta.theta());

        NoiseSpec pitch_spec = cfg.noise.pitch_delta_rad;
        const double pitch_scale = 1.0 + cfg.noise.pitch_drift_rate_per_s.stddev * t;
        pitch_spec.stddev *= pitch_scale;
        const double pitch_meas = pitch_delta + sample_noise(pitch_spec);
        sample.pitch_delta_rad = pitch_meas;
        pitch_delta_stats.add(pitch_meas - pitch_delta);

        addVisionMeasurements(sample, pose);

        out.samples[static_cast<size_t>(i)] = sample;
    }

    if (cfg.trajectory == TrajectoryProfile::Complex) {
        const int min_gap =
            std::max(1, static_cast<int>(cfg.sample_rate_hz * 4.0));
        const double dist_thresh = 0.45;
        const int stride = 2;
        const size_t pose_count = out.ground_truth.p.size();
        for (size_t i = static_cast<size_t>(min_gap); i < pose_count; i += stride) {
            const auto& pose_i = out.ground_truth.p[i];
            size_t best_j = pose_count;
            double best_dist = dist_thresh;
            for (size_t j = 0; j + static_cast<size_t>(min_gap) < i; j += stride) {
                const auto& pose_j = out.ground_truth.p[j];
                const double dx = pose_i.translation().x() - pose_j.translation().x();
                const double dy = pose_i.translation().y() - pose_j.translation().y();
                const double dist = std::hypot(dx, dy);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_j = j;
                }
            }
            if (best_j != pose_count) {
                const auto& pose_j = out.ground_truth.p[best_j];
                const gtsam::Pose2 pose2_i(pose_i.translation().x(),
                                           pose_i.translation().y(),
                                           pose_i.rotation().yaw());
                const gtsam::Pose2 pose2_j(pose_j.translation().x(),
                                           pose_j.translation().y(),
                                           pose_j.rotation().yaw());
                const gtsam::Pose2 delta = pose2_j.between(pose2_i);
                const double dx_noise = sample_noise(cfg.noise.loop_translation_m);
                const double dy_noise = sample_noise(cfg.noise.loop_translation_m);
                const double dtheta_noise = sample_noise(cfg.noise.loop_rotation_rad);
                const gtsam::Pose2 measured(delta.x() + dx_noise,
                                            delta.y() + dy_noise,
                                            normalizeAngle(delta.theta() + dtheta_noise));
                out.loop_closures.push_back(
                    LoopClosureMeasurement{best_j, i, measured});
                loop_translation_stats.add(dx_noise);
                loop_translation_stats.add(dy_noise);
                loop_rotation_stats.add(dtheta_noise);
            }
        }
    }

    out.noise_stats.wheel_delta_m = wheel_delta_stats.stats();
    out.noise_stats.wheel_angle_rad = wheel_angle_stats.stats();
    out.noise_stats.vision_range_m = vision_range_stats.stats();
    out.noise_stats.vision_bearing_rad = vision_bearing_stats.stats();
    out.noise_stats.vision_pixel_px = vision_pixel_stats.stats();
    out.noise_stats.loop_translation_m = loop_translation_stats.stats();
    out.noise_stats.loop_rotation_rad = loop_rotation_stats.stats();
    out.noise_stats.gyro_delta_rad = gyro_delta_stats.stats();
    out.noise_stats.pitch_delta_rad = pitch_delta_stats.stats();

    auto sigma_from_var = [](double variance) {
        return std::sqrt(std::max(variance, 0.0));
    };

    out.sigmas.wheel_delta_m = sigma_from_var(out.noise_stats.wheel_delta_m.variance);
    out.sigmas.wheel_angle_rad = sigma_from_var(out.noise_stats.wheel_angle_rad.variance);
    out.sigmas.vision_range_m = sigma_from_var(out.noise_stats.vision_range_m.variance);
    out.sigmas.vision_bearing_rad = sigma_from_var(out.noise_stats.vision_bearing_rad.variance);
    out.sigmas.loop_translation_m = sigma_from_var(out.noise_stats.loop_translation_m.variance);
    out.sigmas.loop_rotation_rad = sigma_from_var(out.noise_stats.loop_rotation_rad.variance);
    out.sigmas.gyro_delta_rad = sigma_from_var(out.noise_stats.gyro_delta_rad.variance);
    out.sigmas.pitch_delta_rad = sigma_from_var(out.noise_stats.pitch_delta_rad.variance);

    return out;
}
