#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <windows.h>
#include <d2d1_1.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include <mutex>
#include <memory>

#include "renderPacket.h"
#include "..\World\map.h"
#include "..\config.h"

class GraphicsModule {
private:
    ID2D1Factory1* factory = nullptr;
    ID2D1DeviceContext* deviceContext = nullptr;
    ID2D1CommandList* commandList = nullptr;
    ID2D1Image* targetBitmap = nullptr;

    D2D1_RECT_F sensorClip;
    D2D1_RECT_F slamClip;

    std::unordered_map<int, ID2D1SolidColorBrush*> brushes;

    //wall stuff
    D2D1_RECT_F* wallColorSources = nullptr;
    ID2D1Bitmap* whitePixelBitmap = nullptr;

    HWND hwnd = NULL;
    RenderPacket* currentPacket = nullptr;

    std::vector<float>** renderMapAddress = nullptr;
    std::shared_ptr<std::mutex> guardRenderMap;

public:
    GraphicsModule(HINSTANCE hInstance, int nCmdShow);
    ~GraphicsModule();

    void RenderFrame();
    void CreateBackground(Map* map);

    void UpdateRenderInfo(RenderPacket* incoming);

    void DrawRobot(int quadrant, double x, double y, double theta, bool varsInScreenForm);
    void DrawPointCloud();
    void DrawMap();
    void DrawPoses();

    void GiveRenderMapAddress(std::vector<float>** address);
    void GiveRenderMapGuard(std::shared_ptr<std::mutex> guard);

private:
    void InitD2D();
    void CleanupD2D();

    void CreateWindowModule(HINSTANCE hInstance, int nCmdShow);

    void DrawStaticElements();

    std::pair<float, float> XYToDipsBackground(int quadrant, double x, double y);
};

#endif