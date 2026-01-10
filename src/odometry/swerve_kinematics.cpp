#include "odometry/swerve_kinematics.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose2.h>

#include <cmath>

gtsam::Vector3 solveTwistLeastSquaresFromDeltas(const std::array<double, 4>& wheel_delta_m,
                                                const std::array<double, 4>& wheel_angle_rad,
                                                const SwerveParams& p,
                                                double dt) {
    if (dt <= 0.0) {
        return gtsam::Vector3::Zero();
    }
    gtsam::Matrix A(8, 3);
    gtsam::Vector b(8);
    for (int i = 0; i < 4; ++i) {
        const size_t idx = static_cast<size_t>(i);
        const auto [rx, ry] = p.module_pos[idx];
        const double v = wheel_delta_m[idx] / dt;
        const double a = wheel_angle_rad[idx];
        const double vix = v * std::cos(a);
        const double viy = v * std::sin(a);
        A(2 * i + 0, 0) = 1.0;
        A(2 * i + 0, 1) = 0.0;
        A(2 * i + 0, 2) = -ry;
        b(2 * i + 0) = vix;
        A(2 * i + 1, 0) = 0.0;
        A(2 * i + 1, 1) = 1.0;
        A(2 * i + 1, 2) = rx;
        b(2 * i + 1) = viy;
    }
    return A.colPivHouseholderQr().solve(b);
}

gtsam::Pose2 deltaPoseFromWheelDeltas(const std::array<double, 4>& wheel_delta_m,
                                      const std::array<double, 4>& wheel_angle_rad,
                                      const SwerveParams& p,
                                      double dt) {
    if (dt <= 0.0) {
        return gtsam::Pose2(0.0, 0.0, 0.0);
    }
    const gtsam::Vector3 twist =
        solveTwistLeastSquaresFromDeltas(wheel_delta_m, wheel_angle_rad, p, dt);
    return gtsam::Pose2(twist(0) * dt, twist(1) * dt, twist(2) * dt);
}
