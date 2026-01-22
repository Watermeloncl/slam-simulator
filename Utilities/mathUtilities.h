#ifndef UTILITIES_MATH_UTILITIES_H_
#define UTILITIES_MATH_UTILITIES_H_

#include <utility>

// All theta's are expected in radians.

class MathUtilities {
public:
    static double PI;

    static std::pair<double, double> PolarToCartesian(double range, double theta, double cx, double cy);
private:
    MathUtilities() = default;
};

#endif