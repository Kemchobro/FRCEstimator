#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

#include <array>
#include <utility>

struct SwerveParams {
    double wheel_radius_m = 0.0508;
    double drive_gear_ratio = 6.48;
    std::array<std::pair<double, double>, 4> module_pos = {
        std::pair<double, double>{0.314325, 0.31115},
        std::pair<double, double>{0.314325, -0.31115},
        std::pair<double, double>{-0.314325, 0.31115},
        std::pair<double, double>{-0.314325, -0.31115},
    };
};

gtsam::Vector3 solveTwistLeastSquaresFromDeltas(const std::array<double, 4>& wheel_delta_m,
                                                const std::array<double, 4>& wheel_angle_rad,
                                                const SwerveParams& p,
                                                double dt);
gtsam::Pose2 deltaPoseFromWheelDeltas(const std::array<double, 4>& wheel_delta_m,
                                      const std::array<double, 4>& wheel_angle_rad,
                                      const SwerveParams& p,
                                      double dt);
