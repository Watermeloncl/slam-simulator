#ifndef GRAPHICS_RENDER_PACKET_H_
#define GRAPHICS_RENDER_PACKET_H_

#include "..\World\Objects\opoint.h"

struct RenderPacket {
public:
    float realX;
    float realY;
    float realTheta;

    OPoint** pointCloud;

    RenderPacket(float realX, float realY, float realTheta, OPoint** pointCloud);
    ~RenderPacket();
};

#endif