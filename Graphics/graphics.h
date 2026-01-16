#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <windows.h>
#include <d2d1_1.h>
#include <unordered_map>

#include "..\config.h"

class GraphicsModule {
private:
    ID2D1Factory1* factory = nullptr;
    ID2D1DeviceContext* deviceContext = nullptr;
    ID2D1CommandList* commandList = nullptr;
    ID2D1Image* targetBitmap = nullptr;

    std::unordered_map<int, ID2D1SolidColorBrush*> brushes;

    HWND hwnd = NULL;

public:
    GraphicsModule(HINSTANCE hInstance, int nCmdShow);
    ~GraphicsModule();

    void RenderFrame();

private:
    void InitD2D();
    void CleanupD2D();

    void CreateWindowModule(HINSTANCE hInstance, int nCmdShow);

    void RenderBackground();
    void CreateResources();
};

#endif