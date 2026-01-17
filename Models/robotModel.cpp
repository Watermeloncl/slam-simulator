#include "robotModel.h"
#include "sensorModel.h"
#include "motionModel.h"

RobotModel::RobotModel() {
    this->sensorModel = new SensorModel();
    this->motionModel = new MotionModel();
}

RobotModel::~RobotModel() {
    delete this->sensorModel;
    delete this->motionModel;
}
