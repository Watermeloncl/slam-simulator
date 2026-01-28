#ifndef GRAPHICS_RENDER_PACKET_H_
#define GRAPHICS_RENDER_PACKET_H_

#include "..\World\Objects\opoint.h"

struct RenderPacket {
public:
    double realX;
    double realY;
    double realTheta;

    OPoint** pointCloud;

    double poseX;
    double poseY;
    double poseTheta;

    RenderPacket(double realX, double realY, double realTheta, OPoint** pointCloud, double poseX, double poseY, double poseTheta);
    ~RenderPacket();
};

#endif