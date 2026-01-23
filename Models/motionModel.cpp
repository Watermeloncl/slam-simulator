#include <cmath>
#include <thread>
#include <mutex>

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

double MotionModel::GetRealX() {
    return this->realX;
}

double MotionModel::GetRealY() {
    std::lock_guard<std::mutex> lock(this->guardY);
    return this->realY;
}

double MotionModel::GetRealTheta() {
    std::lock_guard<std::mutex> lock(this->guardTheta);
    return this->realTheta;
}

void MotionModel::SetRealX(double x) {
    std::lock_guard<std::mutex> lock(this->guardX);
    this->realX += x;
}

void MotionModel::SetRealY(double y) {
    std::lock_guard<std::mutex> lock(this->guardY);
    this->realY += y;
}

void MotionModel::SetRealTheta(double theta) {
    std::lock_guard<std::mutex> lock(this->guardTheta);
    this->realTheta += theta;
}

void MotionModel::SetStartPosition(double x, double y, double theta) {
    this->realX = x;
    this->realY = y;
    this->realTheta = theta;
}

void MotionModel::DummyUpdate() {
    this->SetRealTheta(0.1745329);
    this->SetRealX(54.0 * cos(this->GetRealTheta()));
    this->SetRealY(54.0 * sin(this->GetRealTheta()));
}
