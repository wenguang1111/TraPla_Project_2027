#ifndef FOP_CUBIC_SPLINE_H
#define FOP_CUBIC_SPLINE_H

#include <utility>
#include <vector>

namespace fop {

class CubicSpline1D {
public:
    CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y);
    double position(double x) const;
    double firstDerivative(double x) const;
    double secondDerivative(double x) const;

private:
    std::size_t searchIndex(double x) const;
    std::vector<double> x_;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
};

class CubicSpline2D {
public:
    CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y);
    std::pair<double, double> position(double s) const;
    double yaw(double s) const;
    double curvature(double s) const;
    double length() const;

private:
    std::vector<double> s_;
    CubicSpline1D sx_;
    CubicSpline1D sy_;
};

}

#endif
