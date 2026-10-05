#ifndef FOP_COST_H
#define FOP_COST_H

#include "fop/types.h"

namespace fop {

class CostFunction {
public:
    double evaluate(const FrenetTrajectory& trajectory, double target_speed, const ObstaclePredictions& obstacles, int time_step_now = 0) const;

private:
    double obstacleCost(const FrenetTrajectory& trajectory, const ObstaclePredictions& obstacles, int time_step_now) const;
    double w_time_ = 0.05;
    double w_speed_ = 0.30;
    double w_accel_ = 0.10;
    double w_jerk_ = 0.05;
    double w_obstacle_ = 0.35;
    double w_offset_ = 0.15;
    double speed_ref_ = 14.0;
    double accel_ref_ = 11.5;
    double jerk_ref_ = 10.0;
    double offset_ref_ = 3.5;
    double time_ref_ = 10.0;
    double safe_distance_ = 8.0;
};

}

#endif
