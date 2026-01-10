#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

struct EkfNoise {
    double process_sigma_x = 0.02;
    double process_sigma_y = 0.02;
    double process_sigma_theta = 0.01;
    double range_sigma = 0.2;
    double bearing_sigma = 0.05;
};

class EkfLocalization {
public:
    EkfLocalization(const gtsam::Pose2& initial_pose, const gtsam::Matrix33& initial_cov);

    void predict(double vx, double vy, double omega, double dt, const EkfNoise& noise);
    void updateVision(const gtsam::Point2& tag_pos,
                      double range_m,
                      double bearing_rad,
                      const EkfNoise& noise);

    const gtsam::Pose2& pose() const;
    const gtsam::Matrix33& covariance() const;

private:
    gtsam::Pose2 pose_;
    gtsam::Matrix33 cov_;

    static double normalizeAngle(double a);
};
