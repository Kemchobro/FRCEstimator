#include "analysis/trajectory_metrics.h"

#include <algorithm>
#include <cmath>
#include <limits>

DeviationStats computeDeviationStats(const Track2D& reference, const Track2D& estimate) {
    DeviationStats out;
    const size_t n = std::min(reference.p.size(), estimate.p.size());
    if (n == 0) {
        return out;
    }

    double sum = 0.0;
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = 0.0;

    for (size_t i = 0; i < n; ++i) {
        const double dx =
            reference.p[i].translation().x() - estimate.p[i].translation().x();
        const double dy =
            reference.p[i].translation().y() - estimate.p[i].translation().y();
        const double dist = std::hypot(dx, dy);
        sum += dist;
        min_v = std::min(min_v, dist);
        max_v = std::max(max_v, dist);
    }

    out.min_m = std::isfinite(min_v) ? min_v : 0.0;
    out.max_m = max_v;
    out.mean_m = sum / static_cast<double>(n);
    return out;
}
