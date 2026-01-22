#ifndef MODELS_MOTION_MODEL_H_
#define MODELS_MOTION_MODEL_H_

#include "..\World\map.h"

class MotionModel {
public:
private:
    Map* map = nullptr;

    double prevX = 0;
    double prevY = 0;
    double prevTheta = 0;

    double realX = 0;
    double realY = 0;
    double realTheta = 0;

public:
    MotionModel();
    ~MotionModel();

    void GiveMap(Map* map);

    double GetPrevX();
    double GetPrevY();
    double GetPrevTheta();

    double GetRealX();
    double GetRealY();
    double GetRealTheta();
    
    void SetStartPosition(double x, double y, double theta);

    void DummyUpdate();
private:

};

#endif