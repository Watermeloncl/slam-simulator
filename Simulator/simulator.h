#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <windows.h>
#include "..\Graphics\graphics.h"

class Simulator {
public:
    Simulator(HINSTANCE hInstance, int nCmdShow);
    ~Simulator();
private:
    GraphicsModule* graphicsModule = NULL;
public:
    void RunMainLoop();
private:

};

#endif