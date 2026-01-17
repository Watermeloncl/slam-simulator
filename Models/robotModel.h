#ifndef MODELS_ROBOT_MODEL_H_
#define MODELS_ROBOT_MODEL_H_

#include "..\World\map.h"
#include "Lidar\sensorModel.h"
#include "Lidar\pointCloud.h"
#include "..\World\Objects\opoint.h"
#include "motionModel.h"

class RobotModel {
public:
private:
    SensorModel* sensorModel = nullptr;
    MotionModel* motionModel = nullptr;

    // map solely for simulation purposes
    Map* map = nullptr;
    
public:
    RobotModel();
    ~RobotModel();

    void InitializeRobot(Map* map);

    float GetRealX();
    float GetRealY();
    float GetRealTheta();

    PointCloud* GetScan();
    OPoint** GetRenderScan();

    void DummyUpdate();
private:

};

#endif