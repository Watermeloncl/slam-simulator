#include <cmath>

#include "mathUtilities.h"

// All theta's are expected in radians.

double MathUtilities::PI = 3.1415926535898;

std::pair<double, double> MathUtilities::PolarToCartesian(double range, double theta, double cx, double cy) {
    return {
        cx + (range * std::cos(theta)),
        cy + (range * std::sin(theta))
    };
}