#include "renderPacket.h"
#include "..\World\Objects\opoint.h"
#include "..\config.h"

RenderPacket::RenderPacket(double realX, double realY, double realTheta, OPoint** pointCloud, double poseX, double poseY, double poseTheta) {
    this->realX = realX;
    this->realY = realY;
    this->realTheta = realTheta;

    this->pointCloud = pointCloud;

    this->poseX = poseX;
    this->poseY = poseY;
    this->poseTheta = poseTheta;
}

// does not currently delete grid
RenderPacket::~RenderPacket() {
    if(this->pointCloud != nullptr) {
        for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
            delete this->pointCloud[i];
        }
        
        delete[] this->pointCloud;
    }
}
