#ifndef MODELS_LIDAR_POLAR_POINT_H_
#define MODELS_LIDAR_POLAR_POINT_H_

struct PolarPoint {
public:
    float range;
    float theta;

    PolarPoint(float range, float theta);
    ~PolarPoint();

    void Print();
};

#endif