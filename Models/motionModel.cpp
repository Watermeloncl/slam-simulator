#include <cmath>
#include <thread>
#include <mutex>

#include "motionModel.h"
#include "..\Utilities\utilities.h"
#include "..\World\map.h"
#include "..\config.h"

MotionModel::MotionModel() {

}

MotionModel::~MotionModel() {
    //do not delete map here.
}

void MotionModel::GiveMap(Map* map) {
    this->map = map;
}

double MotionModel::GetRealX() {
    return this->realX;
}

double MotionModel::GetRealY() {
    return this->realY;
}

double MotionModel::GetRealTheta() {
    return this->realTheta;
}

void MotionModel::ChangeRealX(double x) {
    this->realX += x;
}

void MotionModel::ChangeRealY(double y) {
    this->realY += y;
}

void MotionModel::ChangeRealTheta(double theta) {
    this->realTheta += theta;
}

void MotionModel::SetStartPosition(double x, double y, double theta) {
    this->realX = x;
    this->realY = y;
    this->realTheta = theta;
}

void MotionModel::UpdateRobotPosition(RobotCommand command) {
    //TODO
    //walls were watched out for in commandrobot, but you still
    //  need to go up to it

    //current code ignores walls :)

    double dist, velocityFinal, timeLeft;

    switch(command) {
        case RobotCommand::FORWARD:
            this->ChangeRealTheta(Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION));
            
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

            this->ChangeRealX(dist * cos(this->GetRealTheta()));
            this->ChangeRealY(dist * sin(this->GetRealTheta()));

            break;

        case RobotCommand::STOP:

            this->ChangeRealTheta(Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION));

            if((velocity - MOTION_MODEL_ACCELERATION) > 0) {
                velocityFinal = velocity - MOTION_MODEL_ACCELERATION;

                dist = ((velocityFinal*velocityFinal) - (velocity*velocity)) / (2*-MOTION_MODEL_ACCELERATION);
                velocity = velocityFinal;
            } else {
                dist = (0 - (velocity*velocity)) / (2*-MOTION_MODEL_ACCELERATION);

                velocity = 0;
            }

            dist += Utilities::GetRandomNoise(dist, MOTION_MODEL_FORWARD_DEVIATION);
            this->ChangeRealX(dist * cos(this->GetRealTheta()));
            this->ChangeRealY(dist * sin(this->GetRealTheta()));

            break;

        case RobotCommand::RIGHT:
            this->ChangeRealTheta(-(MOTION_MODEL_ROTATION + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED)));
            break;

        case RobotCommand::LEFT:
            this->ChangeRealTheta(MOTION_MODEL_ROTATION + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED));
            break;
    }

}