#include <windows.h>
#include <iostream>
#include <chrono>

#include "Simulator\simulator.h"
#include "config.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    Simulator* sim = new Simulator(hInstance, nCmdShow);
    sim->RunMainLoop();
    return 0;
}
