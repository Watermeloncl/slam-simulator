#ifndef MODELS_ROBOT_MODEL_H_
#define MODELS_ROBOT_MODEL_H_

#include "sensorModel.h"
#include "motionModel.h"

class RobotModel {
public:
private:
    SensorModel* sensorModel = nullptr;
    MotionModel* motionModel = nullptr;
    
public:
    RobotModel();
    ~RobotModel();
private:

};

#endif