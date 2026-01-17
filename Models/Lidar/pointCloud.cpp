#include <iostream>

#include "pointCloud.h"
#include "polarPoint.h"
#include "..\..\config.h"

PointCloud::PointCloud() {
    this->cloud = new PolarPoint*[SENSOR_MODEL_POINTS_PER_SCAN];
    this->cloudSize = 0;
}

PointCloud::~PointCloud() {
    for(int i = 0; i < this->cloudSize; i++) {
        delete this->cloud[i];
    }
    delete[] this->cloud;
}

void PointCloud::Add(float range, float theta) {
    if(range < SENSOR_MODEL_MEASURE_MIN || range > SENSOR_MODEL_MEASURE_MAX) {
        // out of range
        return;
    }

    this->cloud[this->cloudSize] = new PolarPoint(range, theta);
    (this->cloudSize)++;
}

void PointCloud::Print() {
    std::cout << "Point Cloud of " << this->cloudSize << std::endl;
    for(int i = 0; i < this->cloudSize; i++) {
        this->cloud[i]->Print();
        std::cout << std::endl;
    }
}