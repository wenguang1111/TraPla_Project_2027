#ifndef FOP_TYPES_H
#define FOP_TYPES_H

#include <vector>

namespace fop {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

using Polygon = std::vector<Point2D>;
using ObstaclePredictions = std::vector<std::vector<Polygon>>;

struct SamplingParam {
    double d = 0.0;
    double s_d = 0.0;
    double t = 0.0;
};

struct FrenetState {
    double s = 0.0;
    double s_d = 0.0;
    double s_dd = 0.0;
    double d = 0.0;
    double d_d = 0.0;
    double d_dd = 0.0;
};

struct FrenetTrajectory {
    SamplingParam sampling_param;
    std::vector<double> t;
    std::vector<double> s;
    std::vector<double> s_d;
    std::vector<double> s_dd;
    std::vector<double> s_ddd;
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> d_dd;
    std::vector<double> d_ddd;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> ds;
    std::vector<double> curvature;
    double cost = 0.0;
    bool constraint_passed = false;
    bool collision_passed = false;
};

struct PlannerSettings {
    double tick_t = 0.1;
    double road_width = 7.0;
    int num_width = 5;
    double lowest_speed = 6.0;
    double highest_speed = 12.0;
    int num_speed = 5;
    double min_t = 3.0;
    double max_t = 5.0;
    int num_t = 5;
    bool check_obstacle = true;
    bool check_boundary = true;
};

struct VehicleParams {
    double length = 4.5;
    double width = 1.8;
    double max_speed = 15.0;
    double max_accel = 4.0;
    double max_curvature = 0.35;
};

struct PlanResult {
    FrenetTrajectory best;
    std::vector<FrenetTrajectory> generated;
    std::vector<FrenetTrajectory> feasible;
    bool success = false;
};

}

#endif
