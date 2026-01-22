#include <cmath>

#include "motionModel.h"
#include "..\World\map.h"

MotionModel::MotionModel() {

}

MotionModel::~MotionModel() {
    //do not delete map here.
}

void MotionModel::GiveMap(Map* map) {
    this->map = map;
}

double MotionModel::GetPrevX() {
    return this->prevX;
}

double MotionModel::GetPrevY() {
    return this->prevY;
}

double MotionModel::GetPrevTheta() {
    return this->prevTheta;
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

void MotionModel::SetStartPosition(double x, double y, double theta) {
    this->realX = x;
    this->prevX = x;
    this->realY = y;
    this->prevY = y;
    this->realTheta = theta;
    this->prevTheta = theta;
}

void MotionModel::DummyUpdate() {
    this->prevTheta = this->realTheta;
    this->realTheta += 0.1745329F;

    this->prevX = this->realX;
    this->prevY = this->realY;

    this->realX += (54.0F * (double)cos(this->realTheta));
    this->realY += (54.0F * (double)sin(this->realTheta));
}
