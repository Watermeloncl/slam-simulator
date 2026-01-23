#include <windows.h>
#include <iostream>
#include <thread>

#include "robotModel.h"
#include "Lidar\sensorModel.h"
#include "Lidar\pointCloud.h"
#include "Lidar\sensorPacket.h"
#include "motionModel.h"
#include "..\World\map.h"
#include "..\World\Objects\opoint.h"
#include "..\Utilities\utilities.h"
#include "..\config.h"

RobotModel::RobotModel() {
    this->sensorModel = new SensorModel();
    this->motionModel = new MotionModel();
}

RobotModel::~RobotModel() {
    delete this->motionModel;
    delete this->sensorModel;

    //Do not delete map; (still used and) deleted elsewhere
}

void RobotModel::InitializeRobot(Map* map) {
    this->map = map;
    this->sensorModel->GiveMap(map);
    this->motionModel->GiveMap(map);
    this->sensorModel->GiveMotionModel(this->motionModel);

    int startIndex = Utilities::GetRandomInt(0, map->GetStartsSize() - 1);

    this->motionModel->SetStartPosition(
        map->GetStart(startIndex)->x,
        map->GetStart(startIndex)->y,
        (double)Utilities::GetRandomInt(0, 6)
    );

    this->sensorModel->InitSensor();
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

void RobotModel::KickOffScan(double timestamp) {
    this->sensorModel->SetKickTimeStamp(timestamp);
    ReleaseSemaphore(this->sensorModel->GetSensorSemaphore(), 1, NULL);
}

PointCloud* RobotModel::CopyLatestScan() {
    PointCloud* cloud = this->sensorModel->GetLatestPacket()->cloud;
    if(cloud == nullptr) {
        return nullptr;
    } else {
        return cloud->Copy();
    }
}

OPoint** RobotModel::CopyLatestRenderScan() {
    SensorPacket* packet = this->sensorModel->GetLatestPacket();

    if(packet->renderCloud == nullptr) {
        return nullptr;
    }

    OPoint** cloud = packet->renderCloud;
    OPoint** newCloud = new OPoint*[SENSOR_MODEL_POINTS_PER_SCAN];

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        if(cloud[i] == nullptr) {
            newCloud[i] = nullptr;
        } else {
            newCloud[i] = cloud[i]->Copy();
        }
    }

    return newCloud;
}

void RobotModel::DummyUpdate() {
    this->motionModel->DummyUpdate();
}
