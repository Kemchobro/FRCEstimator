#pragma once

#include <gtsam/geometry/Pose3.h>

#include <vector>

struct Track2D {
    std::vector<double> t;
    std::vector<gtsam::Pose3> p;
};
