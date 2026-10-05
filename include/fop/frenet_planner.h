#ifndef FOP_FRENET_PLANNER_H
#define FOP_FRENET_PLANNER_H

#include "fop/cost.h"
#include "fop/cubic_spline.h"
#include "fop/types.h"

#include <memory>
#include <vector>

namespace fop {

class FrenetPlanner {
public:
    FrenetPlanner(PlannerSettings settings, VehicleParams vehicle);
    void setReferencePath(const std::vector<double>& x, const std::vector<double>& y);
    void setObstacles(ObstaclePredictions obstacles);
    std::vector<SamplingParam> getSamples() const;
    std::vector<FrenetTrajectory> calcFrenetPaths(const FrenetState& state, const std::vector<SamplingParam>& samples) const;
    std::vector<FrenetTrajectory> calcGlobalPaths(const std::vector<FrenetTrajectory>& trajectories) const;
    std::vector<FrenetTrajectory> checkConstraints(const std::vector<FrenetTrajectory>& trajectories) const;
    std::vector<FrenetTrajectory> checkCollisions(const std::vector<FrenetTrajectory>& trajectories, int time_step_now = 0) const;
    void evaluateCosts(std::vector<FrenetTrajectory>& trajectories, double target_speed, int time_step_now = 0) const;
    PlanResult plan(const FrenetState& state, double target_speed, int time_step_now = 0) const;

private:
    PlannerSettings settings_;
    VehicleParams vehicle_;
    CostFunction cost_;
    std::unique_ptr<CubicSpline2D> reference_;
    ObstaclePredictions obstacles_;
};

}

#endif
