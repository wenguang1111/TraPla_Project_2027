#include "fop/cubic_spline.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fop {
namespace {

std::vector<double> cumulativeDistance(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument("reference path requires at least two points");
    }
    std::vector<double> s(x.size(), 0.0);
    for (std::size_t i = 1; i < x.size(); ++i) {
        const double dx = x[i] - x[i - 1];
        const double dy = y[i] - y[i - 1];
        const double ds = std::hypot(dx, dy);
        if (ds <= 1e-9) {
            throw std::invalid_argument("reference path contains duplicate points");
        }
        s[i] = s[i - 1] + ds;
    }
    return s;
}

}

CubicSpline1D::CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) : x_(x), a_(y) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument("spline input size mismatch");
    }
    const std::size_t n = x.size();
    std::vector<double> h(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        h[i] = x_[i + 1] - x_[i];
        if (h[i] <= 0.0) {
            throw std::invalid_argument("spline x values must be increasing");
        }
    }

    c_.assign(n, 0.0);
    if (n > 2) {
        const std::size_t m = n - 2;
        std::vector<double> lower(m, 0.0);
        std::vector<double> diag(m, 0.0);
        std::vector<double> upper(m, 0.0);
        std::vector<double> rhs(m, 0.0);

        for (std::size_t j = 0; j < m; ++j) {
            const std::size_t i = j + 1;
            lower[j] = h[i - 1];
            diag[j] = 2.0 * (h[i - 1] + h[i]);
            upper[j] = h[i];
            rhs[j] = 3.0 * ((a_[i + 1] - a_[i]) / h[i] - (a_[i] - a_[i - 1]) / h[i - 1]);
        }
        lower[0] = 0.0;
        upper[m - 1] = 0.0;

        for (std::size_t i = 1; i < m; ++i) {
            const double factor = lower[i] / diag[i - 1];
            diag[i] -= factor * upper[i - 1];
            rhs[i] -= factor * rhs[i - 1];
        }

        std::vector<double> inner(m, 0.0);
        inner[m - 1] = rhs[m - 1] / diag[m - 1];
        for (std::size_t i = m - 1; i-- > 0;) {
            inner[i] = (rhs[i] - upper[i] * inner[i + 1]) / diag[i];
        }
        for (std::size_t i = 0; i < m; ++i) {
            c_[i + 1] = inner[i];
        }
    }

    b_.resize(n - 1);
    d_.resize(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        b_[i] = (a_[i + 1] - a_[i]) / h[i] - h[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0;
        d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h[i]);
    }
}

std::size_t CubicSpline1D::searchIndex(double x) const {
    if (x <= x_.front()) {
        return 0;
    }
    if (x >= x_.back()) {
        return x_.size() - 2;
    }
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    return static_cast<std::size_t>(std::distance(x_.begin(), it) - 1);
}

double CubicSpline1D::position(double x) const {
    if (x < x_.front() || x > x_.back()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t i = searchIndex(x);
    const double dx = x - x_[i];
    return a_[i] + dx * (b_[i] + dx * (c_[i] + dx * d_[i]));
}

double CubicSpline1D::firstDerivative(double x) const {
    if (x < x_.front() || x > x_.back()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t i = searchIndex(x);
    const double dx = x - x_[i];
    return b_[i] + dx * (2.0 * c_[i] + dx * 3.0 * d_[i]);
}

double CubicSpline1D::secondDerivative(double x) const {
    if (x < x_.front() || x > x_.back()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t i = searchIndex(x);
    const double dx = x - x_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dx;
}

CubicSpline2D::CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y)
    : s_(cumulativeDistance(x, y)), sx_(s_, x), sy_(s_, y) {}

std::pair<double, double> CubicSpline2D::position(double s) const {
    return {sx_.position(s), sy_.position(s)};
}

double CubicSpline2D::yaw(double s) const {
    return std::atan2(sy_.firstDerivative(s), sx_.firstDerivative(s));
}

double CubicSpline2D::curvature(double s) const {
    const double dx = sx_.firstDerivative(s);
    const double dy = sy_.firstDerivative(s);
    const double ddx = sx_.secondDerivative(s);
    const double ddy = sy_.secondDerivative(s);
    const double denom = std::pow(dx * dx + dy * dy, 1.5);
    if (denom <= 1e-12) {
        return 0.0;
    }
    return (dx * ddy - dy * ddx) / denom;
}

double CubicSpline2D::length() const {
    return s_.back();
}

}
