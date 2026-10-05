#ifndef FOP_COLLISION_H
#define FOP_COLLISION_H

#include "fop/types.h"

namespace fop {

Polygon vehiclePolygon(double x, double y, double yaw, double length, double width);
bool polygonsCollide(const Polygon& a, const Polygon& b);
bool trajectoryCollisionFree(const FrenetTrajectory& trajectory, const ObstaclePredictions& obstacles, const VehicleParams& vehicle, int time_step_now = 0);

}

#endif
