#ifndef UTILITIES_MATH_UTILITIES_H_
#define UTILITIES_MATH_UTILITIES_H_

#include <utility>

// All theta's are expected in radians.

class MathUtilities {
public:
    static float PI;

    static std::pair<float, float> PolarToCartesian(float range, float theta, float cx, float cy);
private:
    MathUtilities() = default;
};

#endif