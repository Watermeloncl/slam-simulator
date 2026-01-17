#include <cmath>

#include "mathUtilities.h"

// All theta's are expected in radians.

float MathUtilities::PI = 3.1415927F;

std::pair<float, float> MathUtilities::PolarToCartesian(float range, float theta, float cx, float cy) {
    return {
        cx + (range * std::cos(theta)),
        cy + (range * std::sin(theta))
    };
}