#include "renderPacket.h"
#include "..\World\Objects\opoint.h"
#include "..\config.h"

RenderPacket::RenderPacket(double realX, double realY, double realTheta, OPoint** pointCloud) {
    this->realX = realX;
    this->realY = realY;
    this->realTheta = realTheta;

    this->pointCloud = pointCloud;
}

RenderPacket::~RenderPacket() {
    if(this->pointCloud != nullptr) {
        for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
            delete this->pointCloud[i];
        }
        
        delete[] this->pointCloud;
    }
}
