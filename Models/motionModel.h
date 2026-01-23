#ifndef MODELS_MOTION_MODEL_H_
#define MODELS_MOTION_MODEL_H_

#include <mutex>

#include "..\World\map.h"

class MotionModel {
public:
private:
    Map* map = nullptr;

    std::mutex guardX;
    std::mutex guardY;
    std::mutex guardTheta;

    double realX = 0;
    double realY = 0;
    double realTheta = 0;

public:
    MotionModel();
    ~MotionModel();

    void GiveMap(Map* map);

    double GetRealX();
    double GetRealY();
    double GetRealTheta();

    void SetRealX(double x);
    void SetRealY(double y);
    void SetRealTheta(double theta);
    
    void SetStartPosition(double x, double y, double theta);

    void DummyUpdate();
private:

};

#endif