#include "odometry/landmark_odometry_solver.h"

#include "odometry/swerve_kinematics.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <functional>

namespace {

gtsam::Key tagKeyFromIndex(size_t index) {
    return gtsam::Symbol('l', static_cast<uint64_t>(index));
}

gtsam::ISAM2Params makeIsamParams() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    return params;
}

double normalizeAngle(double a) {
    constexpr double kPi = 3.14159265358979323846;
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

gtsam::SharedNoiseModel makeWheelBetweenNoise(const std::array<double, 4>& wheel_delta_m,
                                              const std::array<double, 4>& wheel_angle_rad,
                                              double dt,
                                              const SwerveParams& params,
                                              double sigma_delta,
                                              double sigma_angle) {
    auto clamp_sigma = [](double sigma) {
        return (sigma > 1e-6) ? sigma : 1e-6;
    };
    const double sd = clamp_sigma(sigma_delta);
    const double sa = clamp_sigma(sigma_angle);
    if (dt <= 0.0) {
        return gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(sd, sd, sa));
    }

    gtsam::Matrix A(8, 3);
    gtsam::Matrix W = gtsam::Matrix::Zero(8, 8);
    const double dt2 = dt * dt;
    const double sd2 = sd * sd;
    const double sa2 = sa * sa;
    const double eps = 1e-9;

    for (int i = 0; i < 4; ++i) {
        const size_t idx = static_cast<size_t>(i);
        const auto [rx, ry] = params.module_pos[idx];
        const double delta = wheel_delta_m[idx];
        const double angle = wheel_angle_rad[idx];
        const double v = delta / dt;
        const double c = std::cos(angle);
        const double s = std::sin(angle);

        A(2 * i + 0, 0) = 1.0;
        A(2 * i + 0, 1) = 0.0;
        A(2 * i + 0, 2) = -ry;
        A(2 * i + 1, 0) = 0.0;
        A(2 * i + 1, 1) = 1.0;
        A(2 * i + 1, 2) = rx;

        double var_x = (c * c / dt2) * sd2 + (v * v * s * s) * sa2;
        double var_y = (s * s / dt2) * sd2 + (v * v * c * c) * sa2;
        double cov_xy = (c * s / dt2) * sd2 - (v * v * s * c) * sa2;

        var_x = std::max(var_x, eps);
        var_y = std::max(var_y, eps);
        double det = var_x * var_y - cov_xy * cov_xy;
        if (det < eps) {
            cov_xy = 0.0;
            det = var_x * var_y;
            if (det < eps) {
                det = eps;
            }
        }

        const double inv00 = var_y / det;
        const double inv01 = -cov_xy / det;
        const double inv11 = var_x / det;

        W(2 * i + 0, 2 * i + 0) = inv00;
        W(2 * i + 0, 2 * i + 1) = inv01;
        W(2 * i + 1, 2 * i + 0) = inv01;
        W(2 * i + 1, 2 * i + 1) = inv11;
    }

    const gtsam::Matrix33 info = A.transpose() * W * A;
    double det = info.determinant();
    if (!std::isfinite(det) || det < eps) {
        return gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(sd, sd, sa));
    }

    const gtsam::Matrix33 cov_twist = info.inverse();
    gtsam::Matrix33 cov_delta = cov_twist * (dt * dt);
    cov_delta = 0.5 * (cov_delta + cov_delta.transpose());
    cov_delta(0, 0) = std::max(cov_delta(0, 0), eps);
    cov_delta(1, 1) = std::max(cov_delta(1, 1), eps);
    cov_delta(2, 2) = std::max(cov_delta(2, 2), eps);

    const double sigma_x = std::sqrt(cov_delta(0, 0));
    const double sigma_y = std::sqrt(cov_delta(1, 1));
    const double sigma_theta = std::sqrt(cov_delta(2, 2));

    return gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(sigma_x, sigma_y, sigma_theta));
}

class PlanarBetweenFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
    gtsam::Pose2 measured_;

public:
    PlanarBetweenFactor(gtsam::Key key1,
                        gtsam::Key key2,
                        const gtsam::Pose2& measured,
                        const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1, key2),
          measured_(measured) {}

    gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                                const gtsam::Pose3& p2,
                                gtsam::OptionalMatrixType H1 = nullptr,
                                gtsam::OptionalMatrixType H2 = nullptr) const override {
        std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Pose3&)>
            errorFunction = [&](const gtsam::Pose3& a,
                                const gtsam::Pose3& b) -> gtsam::Vector3 {
            const gtsam::Pose2 a2(a.translation().x(),
                                  a.translation().y(),
                                  a.rotation().yaw());
            const gtsam::Pose2 b2(b.translation().x(),
                                  b.translation().y(),
                                  b.rotation().yaw());
            const gtsam::Pose2 delta = a2.between(b2);
            const gtsam::Pose2 err = measured_.between(delta);
            return gtsam::Pose2::Logmap(err);
        };
        if (H1) {
            *H1 = gtsam::numericalDerivative21(errorFunction, p1, p2);
        }
        if (H2) {
            *H2 = gtsam::numericalDerivative22(errorFunction, p1, p2);
        }
        return errorFunction(p1, p2);
    }
};

}

LandmarkOdometrySolver::LandmarkOdometrySolver(const gtsam::Pose3& initial_pose,
                                               const std::vector<gtsam::Point3>& tag_positions,
                                               const MeasurementNoise& noise)
    : isam_(makeIsamParams()),
      current_pose_key_(1),
      tag_positions_(tag_positions) {
    auto clamp_sigma = [](double sigma) {
        return (sigma > 1e-6) ? sigma : 1e-6;
    };
    wheel_delta_sigma_m_ = clamp_sigma(noise.wheel_delta_sigma_m);
    wheel_angle_sigma_rad_ = clamp_sigma(noise.wheel_angle_sigma_rad);
    vision_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(clamp_sigma(noise.vision_bearing_sigma_rad),
                       clamp_sigma(noise.vision_bearing_sigma_rad),
                       clamp_sigma(noise.vision_range_sigma_m)));
    auto loop_diag = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(clamp_sigma(noise.loop_translation_sigma_m),
                       clamp_sigma(noise.loop_translation_sigma_m),
                       clamp_sigma(noise.loop_rotation_sigma_rad)));
    loop_closure_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        loop_diag);
    weak_pose_prior_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1e-6, 1e-6, 1e6, 1e6, 1e6, 1e-6));
    tag_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(
        3, clamp_sigma(noise.tag_prior_sigma_m));

    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(0.01, 0.01, 0.01, 0.01, 0.01, 0.01));
    new_factors_.addPrior(current_pose_key_, initial_pose, prior_noise);
    new_values_.insert(current_pose_key_, initial_pose);

    update();
} 

void LandmarkOdometrySolver::addWheelOdometry(const std::array<double, 4>& wheel_delta_m,
                                              const std::array<double, 4>& wheel_angle_rad,
                                              double dt,
                                              const SwerveParams& params) {
    gtsam::Key next_key = current_pose_key_ + 1;

    const gtsam::Pose3 current_pose = poseGuessForCurrent();
    const gtsam::Pose2 delta2 =
        deltaPoseFromWheelDeltas(wheel_delta_m, wheel_angle_rad, params, dt);
    const double yaw = current_pose.rotation().yaw();
    const double pitch = current_pose.rotation().pitch();
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);
    const double dx_world = cos_y * delta2.x() - sin_y * delta2.y();
    const double dy_world = sin_y * delta2.x() + cos_y * delta2.y();
    const double next_yaw = normalizeAngle(yaw + delta2.theta());
    const gtsam::Pose3 next_pose(
        gtsam::Rot3::RzRyRx(0.0, pitch, next_yaw),
        gtsam::Point3(current_pose.translation().x() + dx_world,
                      current_pose.translation().y() + dy_world,
                      current_pose.translation().z()));
    new_values_.insert(next_key, next_pose);
    new_factors_.addPrior(next_key, next_pose, weak_pose_prior_);
    const auto wheel_noise = makeWheelBetweenNoise(wheel_delta_m, wheel_angle_rad, dt, params,
                                                   wheel_delta_sigma_m_,
                                                   wheel_angle_sigma_rad_);
    new_factors_.emplace_shared<PlanarBetweenFactor>(
        current_pose_key_, next_key, delta2, wheel_noise);

    current_pose_key_ = next_key;
}

void LandmarkOdometrySolver::addVisionTagMeasurement(size_t tag_index,
                                                     double distance_m,
                                                     double bearing_yaw_rad,
                                                     double bearing_pitch_rad) {
    if (tag_index >= tag_positions_.size()) {
        return;
    }
    const gtsam::Key tag_key = tagKeyFromIndex(tag_index);
    if (initialized_tags_.insert(tag_key).second) {
        const auto& tag_pos = tag_positions_[tag_index];
        new_values_.insert(tag_key, tag_pos);
        new_factors_.addPrior(tag_key, tag_pos, tag_prior_noise_);
    }

    const double cos_pitch = std::cos(bearing_pitch_rad);
    const gtsam::Point3 dir_point(
        cos_pitch * std::cos(bearing_yaw_rad),
        cos_pitch * std::sin(bearing_yaw_rad),
        std::sin(bearing_pitch_rad));
    const gtsam::Unit3 bearing = gtsam::Unit3::FromPoint3(dir_point);
    new_factors_.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(
        current_pose_key_, tag_key, bearing, distance_m, vision_noise_);
}

void LandmarkOdometrySolver::addLoopClosure(gtsam::Key key1,
                                            gtsam::Key key2,
                                            const gtsam::Pose2& delta_pose) {
    new_factors_.emplace_shared<PlanarBetweenFactor>(
        key1, key2, delta_pose, loop_closure_noise_);
}

void LandmarkOdometrySolver::addGyroDelta(double delta_theta_rad, double sigma_rad) {
    if (current_pose_key_ <= 1) {
        return;
    }
    const double clamped_sigma = (sigma_rad > 1e-6) ? sigma_rad : 1e-6;
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1000.0, 1000.0, clamped_sigma, 1000.0, 1000.0, 1000.0));
    const gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(0.0, 0.0, delta_theta_rad),
                             gtsam::Point3(0.0, 0.0, 0.0));
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        current_pose_key_ - 1, current_pose_key_, delta, noise);
}

void LandmarkOdometrySolver::addPitchDelta(double delta_pitch_rad, double sigma_rad) {
    if (current_pose_key_ <= 1) {
        return;
    }
    const double clamped_sigma = (sigma_rad > 1e-6) ? sigma_rad : 1e-6;
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1000.0, clamped_sigma, 1000.0, 1000.0, 1000.0, 1000.0));
    const gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(0.0, delta_pitch_rad, 0.0),
                             gtsam::Point3(0.0, 0.0, 0.0));
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        current_pose_key_ - 1, current_pose_key_, delta, noise);
}

void LandmarkOdometrySolver::update() {
    if (new_factors_.empty() && new_values_.empty()) {
        return;
    }
    isam_.update(new_factors_, new_values_);
    current_estimate_ = isam_.calculateEstimate();
    full_graph_.add(new_factors_);
    full_values_.insert(new_values_);
    new_factors_.resize(0);
    new_values_.clear();
}

gtsam::Key LandmarkOdometrySolver::getCurrentPoseKey() const {
    return current_pose_key_;
}

gtsam::Pose3 LandmarkOdometrySolver::currentPoseEstimate() const {
    return current_estimate_.at<gtsam::Pose3>(current_pose_key_);
}

gtsam::Values LandmarkOdometrySolver::calculateEstimate() const {
    return current_estimate_;
}

gtsam::Pose3 LandmarkOdometrySolver::poseGuessForCurrent() const {
    if (new_values_.exists(current_pose_key_)) {
        return new_values_.at<gtsam::Pose3>(current_pose_key_);
    }
    return current_estimate_.at<gtsam::Pose3>(current_pose_key_);
}
