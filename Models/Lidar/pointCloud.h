#ifndef MODELS_POINT_CLOUD_H_
#define MODELS_POINT_CLOUD_H_

#include "polarPoint.h"

struct PointCloud {
public:
    PolarPoint** cloud = nullptr;
    int cloudSize = -1;

    PointCloud();
    ~PointCloud();

    void Add(float range, float theta);
    void Print();
};

#endif