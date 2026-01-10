#pragma once

#include "common/track.h"

struct DeviationStats {
    double min_m = 0.0;
    double max_m = 0.0;
    double mean_m = 0.0;
};

DeviationStats computeDeviationStats(const Track2D& reference, const Track2D& estimate);
