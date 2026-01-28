#include <cmath>
#include <utility>

#include "mathUtilities.h"
#include "utilities.h"

// All theta's are expected in radians.

double MathUtilities::PI = 3.1415926535898;

std::pair<double, double> MathUtilities::PolarToCartesian(double range, double theta, double cx, double cy) {
    return {
        cx + (range * std::cos(theta)),
        cy + (range * std::sin(theta))
    };
}

void MathUtilities::SampleCommand(RobotCommand command, double currTheta, double velocity, double& changeX, double& changeY, double& changeTheta) {
    //TODO
    //walls were watched out for in commandrobot, but you still
    //  need to go up to it

    //current code ignores walls :)

    double dist, velocityFinal, timeLeft;
    switch(command) {
        case RobotCommand::FORWARD:
            changeTheta = Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
            
            if((velocity + MOTION_MODEL_ACCELERATION) < MOTION_MODEL_MAX_VELOCITY) {
                velocityFinal = velocity + MOTION_MODEL_ACCELERATION;
                dist = ((velocityFinal*velocityFinal) - (velocity*velocity)) / (2*MOTION_MODEL_ACCELERATION);
                velocity += MOTION_MODEL_ACCELERATION;
                
            } else {
                dist = ((MOTION_MODEL_MAX_VELOCITY*MOTION_MODEL_MAX_VELOCITY) - (velocity*velocity)) / (2*MOTION_MODEL_ACCELERATION);
                timeLeft = MOTION_PERIOD - ((MOTION_MODEL_MAX_VELOCITY - velocity) / MOTION_MODEL_ACCELERATION);

                dist += (timeLeft * MOTION_MODEL_MAX_VELOCITY);
                velocity = MOTION_MODEL_MAX_VELOCITY;
            }

            dist += Utilities::GetRandomNoise(dist, MOTION_MODEL_FORWARD_DEVIATION);

            changeX = dist * cos(currTheta + changeTheta);
            changeY = dist * sin(currTheta + changeTheta);

            break;
            
        case RobotCommand::STOP:
            changeTheta = Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION);

            if((velocity - MOTION_MODEL_ACCELERATION) > 0) {
                velocityFinal = velocity - MOTION_MODEL_ACCELERATION;

                dist = ((velocityFinal*velocityFinal) - (velocity*velocity)) / (2*-MOTION_MODEL_ACCELERATION);
                velocity = velocityFinal;
            } else {
                dist = (0 - (velocity*velocity)) / (2*-MOTION_MODEL_ACCELERATION);

                velocity = 0;
            }

            dist += Utilities::GetRandomNoise(dist, MOTION_MODEL_FORWARD_DEVIATION);
            changeX = dist * cos(currTheta + changeTheta);
            changeY = dist * sin(currTheta + changeTheta);

            break;

        case RobotCommand::RIGHT:
            changeTheta = -(MOTION_MODEL_ROTATION + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED));
            break;

        case RobotCommand::LEFT:
            changeTheta = MOTION_MODEL_ROTATION + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED);
            break;
    }
}