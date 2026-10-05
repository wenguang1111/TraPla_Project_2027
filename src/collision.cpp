#include "fop/collision.h"

#include <algorithm>
#include <cmath>

namespace fop {
namespace {

double cross(const Point2D& a, const Point2D& b, const Point2D& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(const Point2D& a, const Point2D& b, const Point2D& p) {
    const double eps = 1e-9;
    return std::abs(cross(a, b, p)) <= eps && p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps && p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
}

bool segmentsIntersect(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& d) {
    const double ab_c = cross(a, b, c);
    const double ab_d = cross(a, b, d);
    const double cd_a = cross(c, d, a);
    const double cd_b = cross(c, d, b);
    if (((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) && ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0))) {
        return true;
    }
    return onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b);
}

bool pointInPolygon(const Point2D& p, const Polygon& poly) {
    bool inside = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const auto& pi = poly[i];
        const auto& pj = poly[j];
        const bool intersect = ((pi.y > p.y) != (pj.y > p.y)) && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

bool aabbOverlap(const Polygon& a, const Polygon& b) {
    double aminx = a.front().x;
    double amaxx = a.front().x;
    double aminy = a.front().y;
    double amaxy = a.front().y;
    for (const auto& p : a) {
        aminx = std::min(aminx, p.x);
        amaxx = std::max(amaxx, p.x);
        aminy = std::min(aminy, p.y);
        amaxy = std::max(amaxy, p.y);
    }
    double bminx = b.front().x;
    double bmaxx = b.front().x;
    double bminy = b.front().y;
    double bmaxy = b.front().y;
    for (const auto& p : b) {
        bminx = std::min(bminx, p.x);
        bmaxx = std::max(bmaxx, p.x);
        bminy = std::min(bminy, p.y);
        bmaxy = std::max(bmaxy, p.y);
    }
    return !(amaxx < bminx || bmaxx < aminx || amaxy < bminy || bmaxy < aminy);
}

}

Polygon vehiclePolygon(double x, double y, double yaw, double length, double width) {
    const double hl = length * 0.5;
    const double hw = width * 0.5;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const Point2D local[4] = {{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}};
    Polygon result;
    result.reserve(4);
    for (const auto& p : local) {
        result.push_back({x + p.x * c - p.y * s, y + p.x * s + p.y * c});
    }
    return result;
}

bool polygonsCollide(const Polygon& a, const Polygon& b) {
    if (a.size() < 3 || b.size() < 3 || !aabbOverlap(a, b)) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        const auto& a0 = a[i];
        const auto& a1 = a[(i + 1) % a.size()];
        for (std::size_t j = 0; j < b.size(); ++j) {
            const auto& b0 = b[j];
            const auto& b1 = b[(j + 1) % b.size()];
            if (segmentsIntersect(a0, a1, b0, b1)) {
                return true;
            }
        }
    }
    return pointInPolygon(a.front(), b) || pointInPolygon(b.front(), a);
}

bool trajectoryCollisionFree(const FrenetTrajectory& trajectory, const ObstaclePredictions& obstacles, const VehicleParams& vehicle, int time_step_now) {
    const std::size_t n = std::min({trajectory.x.size(), trajectory.y.size(), trajectory.yaw.size()});
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t step = static_cast<std::size_t>(time_step_now) + i;
        if (step >= obstacles.size()) {
            break;
        }
        const Polygon ego = vehiclePolygon(trajectory.x[i], trajectory.y[i], trajectory.yaw[i], vehicle.length, vehicle.width);
        for (const auto& obstacle : obstacles[step]) {
            if (polygonsCollide(ego, obstacle)) {
                return false;
            }
        }
    }
    return true;
}

}
