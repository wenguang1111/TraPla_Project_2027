#include "fop/frenet_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

fop::Polygon rectangle(double cx, double cy, double length, double width) {
    const double hl = length * 0.5;
    const double hw = width * 0.5;
    return {{cx + hl, cy + hw}, {cx + hl, cy - hw}, {cx - hl, cy - hw}, {cx - hl, cy + hw}};
}

void writeTrajectories(const std::string& filename, const std::vector<fop::FrenetTrajectory>& trajectories) {
    std::ofstream file(filename);
    file << "trajectory,x,y,cost,feasible\n";
    for (std::size_t id = 0; id < trajectories.size(); ++id) {
        const auto& trajectory = trajectories[id];
        const std::size_t n = std::min(trajectory.x.size(), trajectory.y.size());
        for (std::size_t i = 0; i < n; ++i) {
            file << id << ',' << trajectory.x[i] << ',' << trajectory.y[i] << ',' << trajectory.cost << ',' << (trajectory.collision_passed ? 1 : 0) << '\n';
        }
    }
}

void writeBest(const std::string& filename, const fop::FrenetTrajectory& trajectory) {
    std::ofstream file(filename);
    file << "t,x,y,s,d,speed,accel,curvature,cost\n";
    const std::size_t n = std::min({trajectory.t.size(), trajectory.x.size(), trajectory.y.size(), trajectory.s.size(), trajectory.d.size(), trajectory.s_d.size(), trajectory.s_dd.size(), trajectory.curvature.size()});
    for (std::size_t i = 0; i < n; ++i) {
        file << trajectory.t[i] << ',' << trajectory.x[i] << ',' << trajectory.y[i] << ',' << trajectory.s[i] << ',' << trajectory.d[i] << ',' << trajectory.s_d[i] << ',' << trajectory.s_dd[i] << ',' << trajectory.curvature[i] << ',' << trajectory.cost << '\n';
    }
}

}

int main() {
    fop::PlannerSettings settings;
    settings.num_width = 5;
    settings.num_speed = 5;
    settings.num_t = 5;
    settings.road_width = 7.0;
    settings.lowest_speed = 6.0;
    settings.highest_speed = 12.0;
    settings.min_t = 3.0;
    settings.max_t = 5.0;

    fop::VehicleParams vehicle;
    fop::FrenetPlanner planner(settings, vehicle);

    std::vector<double> ref_x;
    std::vector<double> ref_y;
    for (int i = 0; i <= 60; ++i) {
        const double x = static_cast<double>(i) * 2.0;
        ref_x.push_back(x);
        ref_y.push_back(1.2 * std::sin(x / 22.0));
    }
    planner.setReferencePath(ref_x, ref_y);

    fop::ObstaclePredictions obstacles(80);
    const double obstacle_x = 18.0;
    const double obstacle_y = 1.2 * std::sin(obstacle_x / 22.0);
    for (auto& frame : obstacles) {
        frame.push_back(rectangle(obstacle_x, obstacle_y, 4.5, 1.8));
    }
    planner.setObstacles(obstacles);

    fop::FrenetState state;
    state.s = 0.0;
    state.s_d = 8.0;

    const double target_speed = 10.0;
    const auto start = std::chrono::steady_clock::now();
    const auto result = planner.plan(state, target_speed);
    const auto end = std::chrono::steady_clock::now();
    const double runtime_ms = std::chrono::duration<double, std::milli>(end - start).count();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "generated: " << result.generated.size() << '\n';
    std::cout << "feasible: " << result.feasible.size() << '\n';
    std::cout << "runtime_ms: " << runtime_ms << '\n';

    if (!result.success) {
        std::cout << "planning failed\n";
        return 1;
    }

    std::cout << "best_cost: " << result.best.cost << '\n';
    std::cout << "best_d: " << result.best.sampling_param.d << '\n';
    std::cout << "best_speed: " << result.best.sampling_param.s_d << '\n';
    std::cout << "best_horizon: " << result.best.sampling_param.t << '\n';

    writeTrajectories("trajectories.csv", result.generated);
    writeBest("best_trajectory.csv", result.best);
    return 0;
}
