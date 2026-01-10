#include "ekf/ekf_localization.h"

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

}

EkfLocalization::EkfLocalization(const gtsam::Pose2& initial_pose,
                                 const gtsam::Matrix33& initial_cov)
    : pose_(initial_pose), cov_(initial_cov) {}

double EkfLocalization::normalizeAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

void EkfLocalization::predict(double vx, double vy, double omega, double dt, const EkfNoise& noise) {
    if (dt <= 0.0) {
        return;
    }

    const double theta = pose_.theta();
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);

    const double dx = (cos_t * vx - sin_t * vy) * dt;
    const double dy = (sin_t * vx + cos_t * vy) * dt;
    const double dtheta = omega * dt;

    pose_ = gtsam::Pose2(pose_.x() + dx,
                         pose_.y() + dy,
                         normalizeAngle(theta + dtheta));

    gtsam::Matrix33 F = gtsam::Matrix33::Identity();
    F(0, 2) = (-sin_t * vx - cos_t * vy) * dt;
    F(1, 2) = (cos_t * vx - sin_t * vy) * dt;

    gtsam::Matrix33 Q = gtsam::Matrix33::Zero();
    Q(0, 0) = noise.process_sigma_x * noise.process_sigma_x;
    Q(1, 1) = noise.process_sigma_y * noise.process_sigma_y;
    Q(2, 2) = noise.process_sigma_theta * noise.process_sigma_theta;

    cov_ = F * cov_ * F.transpose() + Q;
}

void EkfLocalization::updateVision(const gtsam::Point2& tag_pos,
                                   double range_m,
                                   double bearing_rad,
                                   const EkfNoise& noise) {
    const double dx = tag_pos.x() - pose_.x();
    const double dy = tag_pos.y() - pose_.y();
    const double q = dx * dx + dy * dy;
    if (q <= 1e-9) {
        return;
    }

    const double r = std::sqrt(q);
    const double bearing_pred = normalizeAngle(std::atan2(dy, dx) - pose_.theta());

    gtsam::Vector2 z;
    z << range_m, bearing_rad;
    gtsam::Vector2 z_hat;
    z_hat << r, bearing_pred;

    gtsam::Vector2 innovation;
    innovation(0) = z(0) - z_hat(0);
    innovation(1) = normalizeAngle(z(1) - z_hat(1));

    gtsam::Matrix23 H;
    H(0, 0) = -dx / r;
    H(0, 1) = -dy / r;
    H(0, 2) = 0.0;
    H(1, 0) = dy / q;
    H(1, 1) = -dx / q;
    H(1, 2) = -1.0;

    gtsam::Matrix22 R = gtsam::Matrix22::Zero();
    R(0, 0) = noise.range_sigma * noise.range_sigma;
    R(1, 1) = noise.bearing_sigma * noise.bearing_sigma;

    gtsam::Matrix22 S = H * cov_ * H.transpose() + R;
    gtsam::Matrix32 K = cov_ * H.transpose() * S.inverse();

    gtsam::Vector3 delta = K * innovation;
    pose_ = gtsam::Pose2(pose_.x() + delta(0),
                         pose_.y() + delta(1),
                         normalizeAngle(pose_.theta() + delta(2)));

    gtsam::Matrix33 I = gtsam::Matrix33::Identity();
    cov_ = (I - K * H) * cov_;
}

const gtsam::Pose2& EkfLocalization::pose() const {
    return pose_;
}

const gtsam::Matrix33& EkfLocalization::covariance() const {
    return cov_;
}
