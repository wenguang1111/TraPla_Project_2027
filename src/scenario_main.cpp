#include "fop/frenet_planner.h"
#include "fop/scenario_io.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void writeGenerated(const std::filesystem::path& path, const std::vector<fop::FrenetTrajectory>& trajectories) {
    std::ofstream file(path);
    file << "trajectory,x,y,cost,constraint_passed,collision_passed\n";
    for (std::size_t id = 0; id < trajectories.size(); ++id) {
        const auto& trajectory = trajectories[id];
        const std::size_t n = std::min(trajectory.x.size(), trajectory.y.size());
        for (std::size_t i = 0; i < n; ++i) {
            file << id << ','
                 << trajectory.x[i] << ','
                 << trajectory.y[i] << ','
                 << trajectory.cost << ','
                 << (trajectory.constraint_passed ? 1 : 0) << ','
                 << (trajectory.collision_passed ? 1 : 0) << '\n';
        }
    }
}

void writeBest(const std::filesystem::path& path, const fop::FrenetTrajectory& trajectory) {
    std::ofstream file(path);
    file << std::setprecision(17);
    file << "t,x,y,s,d,speed,accel,d_speed,d_accel,curvature,cost\n";
    const std::size_t n = std::min({trajectory.t.size(), trajectory.x.size(), trajectory.y.size(), trajectory.s.size(), trajectory.d.size(), trajectory.s_d.size(), trajectory.s_dd.size(), trajectory.d_d.size(), trajectory.d_dd.size(), trajectory.curvature.size()});
    for (std::size_t i = 0; i < n; ++i) {
        file << trajectory.t[i] << ','
             << trajectory.x[i] << ','
             << trajectory.y[i] << ','
             << trajectory.s[i] << ','
             << trajectory.d[i] << ','
             << trajectory.s_d[i] << ','
             << trajectory.s_dd[i] << ','
             << trajectory.d_d[i] << ','
             << trajectory.d_dd[i] << ','
             << trajectory.curvature[i] << ','
             << trajectory.cost << '\n';
    }
}

}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: fop_scenario <input_file> <output_dir>\n";
        return 1;
    }

    try {
        const fop::ScenarioInput input = fop::loadScenarioInput(argv[1]);
        fop::FrenetPlanner planner(input.settings, input.vehicle);
        planner.setReferencePath(input.reference_x, input.reference_y);
        planner.setObstacles(input.obstacles);

        const fop::PlanResult result = planner.plan(input.state, input.target_speed);

        const std::filesystem::path output_dir(argv[2]);
        std::filesystem::create_directories(output_dir);
        writeGenerated(output_dir / "trajectories.csv", result.generated);

        if (!result.success) {
            std::cout << "planning failed\n";
            return 2;
        }

        writeBest(output_dir / "best_trajectory.csv", result.best);
        std::cout << "planning succeeded\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
