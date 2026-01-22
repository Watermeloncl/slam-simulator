#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <windows.h>
#include <thread>

#include "..\Graphics\graphics.h"
#include "..\Graphics\renderPacket.h"
#include "..\World\world.h"
#include "..\Models\robotModel.h"
#include "..\AI\ai.h"
#include "..\SLAMModels\Templates\slam.h"

class Simulator {
private:
    World* world = nullptr;
    RobotModel* robotModel = nullptr;
    AIModule* aiModule = nullptr;
    SLAMModule* slamModule = nullptr;

    std::thread sensorThread;
public:
    Simulator(HINSTANCE hInstance, int nCmdShow);
    ~Simulator();
private:
    GraphicsModule* graphicsModule = nullptr;
public:
    void RunMainLoop();
private:

};

#endif