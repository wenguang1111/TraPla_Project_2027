#include "fop/cost.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fop {
namespace {

double square(double x) {
    return x * x;
}

Point2D centroid(const Polygon& poly) {
    Point2D c;
    if (poly.empty()) {
        return c;
    }
    for (const auto& p : poly) {
        c.x += p.x;
        c.y += p.y;
    }
    c.x /= static_cast<double>(poly.size());
    c.y /= static_cast<double>(poly.size());
    return c;
}

}

double CostFunction::obstacleCost(const FrenetTrajectory& trajectory, const ObstaclePredictions& obstacles, int time_step_now) const {
    double cost = 0.0;
    const std::size_t n = std::min(trajectory.x.size(), trajectory.y.size());
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t step = static_cast<std::size_t>(time_step_now) + i;
        if (step >= obstacles.size() || obstacles[step].empty()) {
            continue;
        }
        double min_distance = std::numeric_limits<double>::infinity();
        for (const auto& obstacle : obstacles[step]) {
            const Point2D c = centroid(obstacle);
            min_distance = std::min(min_distance, std::hypot(trajectory.x[i] - c.x, trajectory.y[i] - c.y));
        }
        const double gap = std::max(0.0, safe_distance_ - min_distance);
        const double normalized = std::min(1.0, gap / safe_distance_);
        cost += normalized * normalized;
    }
    return cost;
}

double CostFunction::evaluate(const FrenetTrajectory& trajectory, double target_speed, const ObstaclePredictions& obstacles, int time_step_now) const {
    if (trajectory.t.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double speed = 0.0;
    double accel = 0.0;
    double jerk = 0.0;
    double offset = 0.0;
    for (double v : trajectory.s_d) {
        speed += square(v - target_speed) / square(speed_ref_);
    }
    for (double a : trajectory.s_dd) {
        accel += square(a) / square(accel_ref_);
    }
    for (double a : trajectory.d_dd) {
        accel += square(a) / square(accel_ref_);
    }
    for (double j : trajectory.s_ddd) {
        jerk += square(j) / square(jerk_ref_);
    }
    for (double j : trajectory.d_ddd) {
        jerk += square(j) / square(jerk_ref_);
    }
    for (double d : trajectory.d) {
        offset += square(d) / square(offset_ref_);
    }
    const double time = trajectory.t.back() / time_ref_;
    const double obstacle = obstacleCost(trajectory, obstacles, time_step_now);
    const double total = w_time_ * time + w_speed_ * speed + w_accel_ * accel + w_jerk_ * jerk + w_obstacle_ * obstacle + w_offset_ * offset;
    return total / static_cast<double>(trajectory.t.size());
}

}
