#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <windows.h>
#include <d2d1.h>

class GraphicsModule {
private:
    ID2D1Factory* factory = nullptr;
    ID2D1HwndRenderTarget* renderTarget = nullptr;
    ID2D1Bitmap* bitmap = nullptr;

    HWND hwnd = NULL;

public:
    GraphicsModule(HINSTANCE hInstance, int nCmdShow);
    ~GraphicsModule();

    void RenderFrame(UINT32* pixels);

private:
    void InitD2D(HWND hwnd);
    void CleanupD2D();

    void CreateWindowModule(HINSTANCE hInstance, int nCmdShow);
};

#endif