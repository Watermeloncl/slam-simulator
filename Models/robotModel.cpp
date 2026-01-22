#include <iostream>

#include "robotModel.h"
#include "Lidar\sensorModel.h"
#include "Lidar\pointCloud.h"
#include "motionModel.h"
#include "..\World\map.h"
#include "..\World\Objects\opoint.h"
#include "..\Utilities\utilities.h"

RobotModel::RobotModel() {
    this->sensorModel = new SensorModel();
    this->motionModel = new MotionModel();
}

RobotModel::~RobotModel() {
    delete this->sensorModel;
    delete this->motionModel;

    //Do not delete map; (still used and) deleted elsewhere
}

void RobotModel::InitializeRobot(Map* map) {
    this->map = map;
    this->sensorModel->GiveMap(map);
    this->motionModel->GiveMap(map);

    int startIndex = Utilities::GetRandomInt(0, map->GetStartsSize() - 1);

    this->motionModel->SetStartPosition(
        map->GetStart(startIndex)->x,
        map->GetStart(startIndex)->y,
        (double)Utilities::GetRandomInt(0, 6)
    );
}

// Rendering function
double RobotModel::GetRealX() {
    return this->motionModel->GetRealX();
}

// Rendering function
double RobotModel::GetRealY() {
    return this->motionModel->GetRealY();
}

// Rendering function
double RobotModel::GetRealTheta() {
    return this->motionModel->GetRealTheta();
}

PointCloud* RobotModel::GetScan() {
    // given the robots current position and where it's going to be,
    // pass that to sensor model and get back a polar point cloud

    return this->sensorModel->GetScan(
        this->motionModel->GetPrevX(),
        this->motionModel->GetPrevY(),
        this->motionModel->GetPrevTheta(),
        this->motionModel->GetRealX(),
        this->motionModel->GetRealY(),
        this->motionModel->GetRealTheta()
    );
}

OPoint** RobotModel::GetRenderScan() {
    return this->sensorModel->GetRenderScan();
}

void RobotModel::DummyUpdate() {
    this->motionModel->DummyUpdate();
}
