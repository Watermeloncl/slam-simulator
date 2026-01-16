#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <windows.h>

#include "..\Graphics\graphics.h"
#include "..\World\world.h"

class Simulator {
private:
    World* world = nullptr;

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