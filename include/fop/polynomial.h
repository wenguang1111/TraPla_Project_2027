#ifndef FOP_POLYNOMIAL_H
#define FOP_POLYNOMIAL_H

namespace fop {

class QuarticPolynomial {
public:
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time);
    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double jerk(double t) const;

private:
    double a0_ = 0.0;
    double a1_ = 0.0;
    double a2_ = 0.0;
    double a3_ = 0.0;
    double a4_ = 0.0;
};

class QuinticPolynomial {
public:
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time);
    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double jerk(double t) const;

private:
    double a0_ = 0.0;
    double a1_ = 0.0;
    double a2_ = 0.0;
    double a3_ = 0.0;
    double a4_ = 0.0;
    double a5_ = 0.0;
};

}

#endif
