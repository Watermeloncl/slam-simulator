#ifndef UTILITIES_MATH_UTILITIES_H_
#define UTILITIES_MATH_UTILITIES_H_

#include <utility>

#include "..\config.h"

// All theta's are expected in radians.

class MathUtilities {
public:
    static double PI;

    static std::pair<double, double> PolarToCartesian(double range, double theta, double cx, double cy);
    static void SampleCommand(RobotCommand command, double currTheta, double velocity, double& changeX, double& changeY, double& changeTheta);
private:
    MathUtilities() = default;
};

#endif