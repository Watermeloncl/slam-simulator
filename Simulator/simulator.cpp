#include <chrono>

#include "simulator.h"
#include "..\config.h"

using SchedClock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;
using TimeStamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

UINT32* g_pixels = new UINT32[CLIENT_SCREEN_WIDTH * CLIENT_SCREEN_HEIGHT];

Simulator::Simulator(HINSTANCE hInstance, int nCmdShow) {
    this->graphicsModule = new GraphicsModule(hInstance, nCmdShow);
}

Simulator::~Simulator() {
    delete this->graphicsModule;
    delete[] g_pixels;
}

void Simulator::RunMainLoop() {
    TimeStamp lastTime = SchedClock::now();
    TimeStamp now = SchedClock::now();

    Duration elapsed;
    double accumulator = 0.0;
    const double dt = 1.0 / 60.0;

    MSG msg = {};

    // int frame = 0;
    bool running = true;

    while(running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) running = false;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        now = SchedClock::now();
        elapsed = now - lastTime;
        lastTime = now;
        accumulator += elapsed.count();

        while(accumulator >= dt) {
            // frame++;
            
            accumulator -= dt;
        }

        this->graphicsModule->RenderFrame(g_pixels);
    }
}