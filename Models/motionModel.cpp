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

float MotionModel::GetPrevX() {
    return this->prevX;
}

float MotionModel::GetPrevY() {
    return this->prevY;
}

float MotionModel::GetPrevTheta() {
    return this->prevTheta;
}

float MotionModel::GetRealX() {
    return this->realX;
}

float MotionModel::GetRealY() {
    return this->realY;
}

float MotionModel::GetRealTheta() {
    return this->realTheta;
}

void MotionModel::SetStartPosition(float x, float y, float theta) {
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

    this->realX += (54.0F * (float)cos(this->realTheta));
    this->realY += (54.0F * (float)sin(this->realTheta));
}
