#ifndef FOP_SCENARIO_IO_H
#define FOP_SCENARIO_IO_H

#include "fop/types.h"

#include <string>
#include <vector>

namespace fop {

struct ScenarioInput {
    PlannerSettings settings;
    VehicleParams vehicle;
    FrenetState state;
    double target_speed = 0.0;
    std::vector<double> reference_x;
    std::vector<double> reference_y;
    ObstaclePredictions obstacles;
};

ScenarioInput loadScenarioInput(const std::string& path);

}

#endif
