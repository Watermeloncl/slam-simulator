#ifndef MODELS_LIDAR_SENSOR_MODEL_H_
#define MODELS_LIDAR_SENSOR_MODEL_H_

// Modelled of SlamTec RP LiDAR-A1
// https://www.slamtec.com/en/Lidar/A1Spec

#include "pointCloud.h"
#include "..\..\World\map.h"
#include "..\..\World\Objects\opoint.h"

class SensorModel {
public:
private:
    Map* map = nullptr;

    OPoint** renderScan = nullptr;
    int currRenderPoints = -1;
public:
    SensorModel();
    ~SensorModel();

    void GiveMap(Map* map);
    PointCloud* GetScan(float prevX, float prevY, float prevTheta, float currX, float currY, float currTheta);
    OPoint** GetRenderScan();
private:
    float GetCollisionDistance(float x, float y, float theta);
    void PrintRenderCloud();
};

#endif