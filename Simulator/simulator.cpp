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

}

void Simulator::RunMainLoop() {
    TimeStamp lastTime = SchedClock::now();
    TimeStamp now = SchedClock::now();

    Duration elapsed;
    double accumulator = 0.0;
    const double dt = 1.0 / 60.0;

    MSG msg = {};

    int frame = 0;
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

        UINT r, g, b;

        while(accumulator >= dt) {
            frame++;

            // pretty colors
            for(int y = 0; y < CLIENT_SCREEN_HEIGHT; y++) {
                for (int x = 0; x < CLIENT_SCREEN_WIDTH; x++) {
                    r = (x + frame) % 256;
                    g = (y + frame) % 256;
                    b = (x + y + frame) % 256;
                    g_pixels[y * CLIENT_SCREEN_WIDTH + x] = 0xFF000000 | (r << 16) | (g << 8) | b;
                }
            }
            
            accumulator -= dt;
        }

        //blocks until the end of the "frame" forcing another frame next time
        this->graphicsModule->RenderFrame(g_pixels);
    }

    delete this->graphicsModule;
    delete[] g_pixels;
}