#include "fop/polynomial.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace fop {
namespace {

std::array<double, 3> solve3x3(std::array<std::array<double, 3>, 3> a, std::array<double, 3> b) {
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::abs(a[row][col]) > std::abs(a[pivot][col])) {
                pivot = row;
            }
        }
        if (std::abs(a[pivot][col]) < 1e-12) {
            throw std::runtime_error("Singular polynomial system");
        }
        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }
        const double inv = 1.0 / a[col][col];
        for (int j = col; j < 3; ++j) {
            a[col][j] *= inv;
        }
        b[col] *= inv;
        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            for (int j = col; j < 3; ++j) {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }
    return b;
}

}

QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time) {
    if (time <= 0.0) {
        throw std::invalid_argument("time must be positive");
    }
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;
    const double t2 = time * time;
    const double t3 = t2 * time;
    const double t4 = t3 * time;
    const double b0 = vxe - a1_ - 2.0 * a2_ * time;
    const double b1 = axe - 2.0 * a2_;
    const double det = 12.0 * t4;
    a3_ = (12.0 * t2 * b0 - 4.0 * t3 * b1) / det;
    a4_ = (3.0 * t2 * b1 - 6.0 * time * b0) / det;
}

double QuarticPolynomial::position(double t) const {
    return a0_ + t * (a1_ + t * (a2_ + t * (a3_ + t * a4_)));
}

double QuarticPolynomial::velocity(double t) const {
    return a1_ + t * (2.0 * a2_ + t * (3.0 * a3_ + t * 4.0 * a4_));
}

double QuarticPolynomial::acceleration(double t) const {
    return 2.0 * a2_ + t * (6.0 * a3_ + t * 12.0 * a4_);
}

double QuarticPolynomial::jerk(double t) const {
    return 6.0 * a3_ + 24.0 * a4_ * t;
}

QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time) {
    if (time <= 0.0) {
        throw std::invalid_argument("time must be positive");
    }
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;
    const double t2 = time * time;
    const double t3 = t2 * time;
    const double t4 = t3 * time;
    const double t5 = t4 * time;
    std::array<std::array<double, 3>, 3> a{{
        {t3, t4, t5},
        {3.0 * t2, 4.0 * t3, 5.0 * t4},
        {6.0 * time, 12.0 * t2, 20.0 * t3}
    }};
    std::array<double, 3> b{{
        xe - a0_ - a1_ * time - a2_ * t2,
        vxe - a1_ - 2.0 * a2_ * time,
        axe - 2.0 * a2_
    }};
    const auto x = solve3x3(a, b);
    a3_ = x[0];
    a4_ = x[1];
    a5_ = x[2];
}

double QuinticPolynomial::position(double t) const {
    return a0_ + t * (a1_ + t * (a2_ + t * (a3_ + t * (a4_ + t * a5_))));
}

double QuinticPolynomial::velocity(double t) const {
    return a1_ + t * (2.0 * a2_ + t * (3.0 * a3_ + t * (4.0 * a4_ + t * 5.0 * a5_)));
}

double QuinticPolynomial::acceleration(double t) const {
    return 2.0 * a2_ + t * (6.0 * a3_ + t * (12.0 * a4_ + t * 20.0 * a5_));
}

double QuinticPolynomial::jerk(double t) const {
    return 6.0 * a3_ + t * (24.0 * a4_ + t * 60.0 * a5_);
}

}
