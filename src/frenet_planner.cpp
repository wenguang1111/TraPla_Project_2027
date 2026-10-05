#include "fop/frenet_planner.h"

#include "fop/collision.h"
#include "fop/polynomial.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fop {
namespace {

std::vector<double> linspace(double start, double end, int count) {
    if (count <= 0) {
        throw std::invalid_argument("sample count must be positive");
    }
    if (count == 1) {
        return {(start + end) * 0.5};
    }
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(count));
    const double step = (end - start) / static_cast<double>(count - 1);
    for (int i = 0; i < count; ++i) {
        values.push_back(start + step * static_cast<double>(i));
    }
    return values;
}

double normalizeAngle(double angle) {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

}

FrenetPlanner::FrenetPlanner(PlannerSettings settings, VehicleParams vehicle) : settings_(settings), vehicle_(vehicle) {}

void FrenetPlanner::setReferencePath(const std::vector<double>& x, const std::vector<double>& y) {
    reference_ = std::make_unique<CubicSpline2D>(x, y);
}

void FrenetPlanner::setObstacles(ObstaclePredictions obstacles) {
    obstacles_ = std::move(obstacles);
}

std::vector<SamplingParam> FrenetPlanner::getSamples() const {
    const double half_range = std::max(0.0, 0.5 * (settings_.road_width - vehicle_.width));
    const auto d_values = linspace(-half_range, half_range, settings_.num_width);
    const auto v_values = linspace(settings_.lowest_speed, settings_.highest_speed, settings_.num_speed);
    const auto t_values = linspace(settings_.min_t, settings_.max_t, settings_.num_t);
    std::vector<SamplingParam> samples;
    samples.reserve(d_values.size() * v_values.size() * t_values.size());
    for (double d : d_values) {
        for (double v : v_values) {
            for (double t : t_values) {
                samples.push_back({d, v, t});
            }
        }
    }
    return samples;
}

std::vector<FrenetTrajectory> FrenetPlanner::calcFrenetPaths(const FrenetState& state, const std::vector<SamplingParam>& samples) const {
    std::vector<FrenetTrajectory> result;
    result.reserve(samples.size());
    for (const auto& sample : samples) {
        const double horizon = std::max(sample.t, settings_.tick_t);
        QuinticPolynomial lateral(state.d, state.d_d, state.d_dd, sample.d, 0.0, 0.0, horizon);
        QuarticPolynomial longitudinal(state.s, state.s_d, state.s_dd, sample.s_d, 0.0, horizon);
        FrenetTrajectory trajectory;
        trajectory.sampling_param = sample;
        for (double t = 0.0; t <= horizon + 1e-9; t += settings_.tick_t) {
            const double ti = std::min(t, horizon);
            trajectory.t.push_back(ti);
            trajectory.d.push_back(lateral.position(ti));
            trajectory.d_d.push_back(lateral.velocity(ti));
            trajectory.d_dd.push_back(lateral.acceleration(ti));
            trajectory.d_ddd.push_back(lateral.jerk(ti));
            trajectory.s.push_back(longitudinal.position(ti));
            trajectory.s_d.push_back(longitudinal.velocity(ti));
            trajectory.s_dd.push_back(longitudinal.acceleration(ti));
            trajectory.s_ddd.push_back(longitudinal.jerk(ti));
            if (ti >= horizon) {
                break;
            }
        }
        result.push_back(std::move(trajectory));
    }
    return result;
}

std::vector<FrenetTrajectory> FrenetPlanner::calcGlobalPaths(const std::vector<FrenetTrajectory>& trajectories) const {
    if (!reference_) {
        throw std::runtime_error("reference path not set");
    }
    constexpr double half_pi = 1.57079632679489661923;
    std::vector<FrenetTrajectory> result;
    result.reserve(trajectories.size());
    for (auto trajectory : trajectories) {
        const std::size_t n = std::min(trajectory.s.size(), trajectory.d.size());
        for (std::size_t i = 0; i < n; ++i) {
            if (trajectory.s[i] < 0.0 || trajectory.s[i] > reference_->length()) {
                break;
            }
            const auto [rx, ry] = reference_->position(trajectory.s[i]);
            if (!std::isfinite(rx) || !std::isfinite(ry)) {
                break;
            }
            const double yaw = reference_->yaw(trajectory.s[i]);
            trajectory.x.push_back(rx + trajectory.d[i] * std::cos(yaw + half_pi));
            trajectory.y.push_back(ry + trajectory.d[i] * std::sin(yaw + half_pi));
        }
        if (trajectory.x.size() != n || trajectory.x.size() < 2) {
            continue;
        }
        for (std::size_t i = 0; i + 1 < trajectory.x.size(); ++i) {
            const double dx = trajectory.x[i + 1] - trajectory.x[i];
            const double dy = trajectory.y[i + 1] - trajectory.y[i];
            trajectory.yaw.push_back(std::atan2(dy, dx));
            trajectory.ds.push_back(std::hypot(dx, dy));
        }
        trajectory.yaw.push_back(trajectory.yaw.back());
        for (std::size_t i = 0; i + 1 < trajectory.yaw.size(); ++i) {
            const double ds = std::max(trajectory.ds[i], 1e-6);
            trajectory.curvature.push_back(normalizeAngle(trajectory.yaw[i + 1] - trajectory.yaw[i]) / ds);
        }
        if (!trajectory.curvature.empty()) {
            trajectory.curvature.push_back(trajectory.curvature.back());
        } else {
            trajectory.curvature.push_back(0.0);
        }
        result.push_back(std::move(trajectory));
    }
    return result;
}

std::vector<FrenetTrajectory> FrenetPlanner::checkConstraints(const std::vector<FrenetTrajectory>& trajectories) const {
    std::vector<FrenetTrajectory> result;
    result.reserve(trajectories.size());
    const double lateral_limit = std::max(0.0, 0.5 * (settings_.road_width - vehicle_.width));
    for (auto trajectory : trajectories) {
        bool valid = true;
        for (double v : trajectory.s_d) {
            if (v < 0.0 || v > vehicle_.max_speed) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            continue;
        }
        for (double a : trajectory.s_dd) {
            if (std::abs(a) > vehicle_.max_accel) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            continue;
        }
        for (double a : trajectory.d_dd) {
            if (std::abs(a) > vehicle_.max_accel) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            continue;
        }
        for (double k : trajectory.curvature) {
            if (std::abs(k) > vehicle_.max_curvature) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            continue;
        }
        if (settings_.check_boundary) {
            for (double d : trajectory.d) {
                if (std::abs(d) > lateral_limit + 1e-9) {
                    valid = false;
                    break;
                }
            }
        }
        if (valid) {
            trajectory.constraint_passed = true;
            result.push_back(std::move(trajectory));
        }
    }
    return result;
}

std::vector<FrenetTrajectory> FrenetPlanner::checkCollisions(const std::vector<FrenetTrajectory>& trajectories, int time_step_now) const {
    if (!settings_.check_obstacle || obstacles_.empty()) {
        auto result = trajectories;
        for (auto& trajectory : result) {
            trajectory.collision_passed = true;
        }
        return result;
    }
    std::vector<FrenetTrajectory> result;
    result.reserve(trajectories.size());
    for (auto trajectory : trajectories) {
        if (trajectoryCollisionFree(trajectory, obstacles_, vehicle_, time_step_now)) {
            trajectory.collision_passed = true;
            result.push_back(std::move(trajectory));
        }
    }
    return result;
}

void FrenetPlanner::evaluateCosts(std::vector<FrenetTrajectory>& trajectories, double target_speed, int time_step_now) const {
    for (auto& trajectory : trajectories) {
        trajectory.cost = cost_.evaluate(trajectory, target_speed, obstacles_, time_step_now);
    }
}

PlanResult FrenetPlanner::plan(const FrenetState& state, double target_speed, int time_step_now) const {
    PlanResult result;
    const auto samples = getSamples();
    const auto frenet = calcFrenetPaths(state, samples);
    result.generated = calcGlobalPaths(frenet);
    const auto constrained = checkConstraints(result.generated);
    result.feasible = checkCollisions(constrained, time_step_now);
    evaluateCosts(result.feasible, target_speed, time_step_now);
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto& trajectory : result.feasible) {
        if (trajectory.cost < best_cost) {
            best_cost = trajectory.cost;
            result.best = trajectory;
            result.success = true;
        }
    }
    return result;
}

}
