#ifndef MODELS_MOTION_MODEL_H_
#define MODELS_MOTION_MODEL_H_

#include "..\World\map.h"

class MotionModel {
public:
private:
    Map* map = nullptr;

    float prevX = 0;
    float prevY = 0;
    float prevTheta = 0;

    float realX = 0;
    float realY = 0;
    float realTheta = 0;

public:
    MotionModel();
    ~MotionModel();

    void GiveMap(Map* map);

    float GetPrevX();
    float GetPrevY();
    float GetPrevTheta();

    float GetRealX();
    float GetRealY();
    float GetRealTheta();
    
    void SetStartPosition(float x, float y, float theta);

    void DummyUpdate();
private:

};

#endif