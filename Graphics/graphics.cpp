#include <windows.h>
#include <d2d1_1.h>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <cmath>

#include "graphics.h"
#include "..\Listener\listener.h"
#include "..\World\map.h"
#include "..\World\Objects\oline.h"
#include "..\World\Objects\opoint.h"
#include "..\Utilities\mathUtilities.h"
// #include "..\Models\Lidar\polarPoint.h"
#include "../config.h"


GraphicsModule::GraphicsModule(HINSTANCE hInstance, int nCmdShow) {
    this->CreateWindowModule(hInstance, nCmdShow);

    this->InitD2D();
}

GraphicsModule::~GraphicsModule() {
    this->CleanupD2D();
    delete this->currentPacket;
}

void GraphicsModule::InitD2D() {
    D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof(ID2D1Factory1), (void**)&factory);

    RECT rc;
    GetClientRect(this->hwnd, &rc);

    ID2D1HwndRenderTarget* hwndRT = nullptr;
    D2D1_RENDER_TARGET_PROPERTIES props = D2D1::RenderTargetProperties();
    D2D1_HWND_RENDER_TARGET_PROPERTIES hwndProps = D2D1::HwndRenderTargetProperties(this->hwnd, D2D1::SizeU(rc.right, rc.bottom));

    factory->CreateHwndRenderTarget(&props, &hwndProps, &hwndRT);

    hwndRT->QueryInterface(__uuidof(ID2D1DeviceContext), (void**)&deviceContext);
    
    this->deviceContext->GetTarget(&this->targetBitmap);

    hwndRT->Release();

    for(int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        ID2D1SolidColorBrush* tempBrush = nullptr;
        this->deviceContext->CreateSolidColorBrush(D2D1::ColorF(COLOR_PALETTE_VALUES[i]), &tempBrush);
        this->brushes[COLOR_PALETTE_VALUES[i]] = tempBrush;
    }

    this->sensorClip = D2D1::RectF(
        0,
        CLIENT_SCREEN_HEIGHT / 2.0,
        CLIENT_SCREEN_WIDTH / 2.0,
        CLIENT_SCREEN_HEIGHT
    );
}

void GraphicsModule::CleanupD2D() {
    for(int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        if(this->brushes[COLOR_PALETTE_VALUES[i]]) this->brushes[COLOR_PALETTE_VALUES[i]]->Release();
    }
    this->brushes.clear();

    if(this->commandList) this->commandList->Release();
    if(this->targetBitmap) this->targetBitmap->Release();
    if (this->deviceContext) this->deviceContext->Release();
    if (this->factory) this->factory->Release();
}

void GraphicsModule::RenderFrame() {
    this->deviceContext->BeginDraw();
    this->deviceContext->Clear(D2D1::ColorF(COLOR_PALETTE_BACKGROUND));

    this->DrawStaticElements();

    //draw dynamic elements. consider push axis aligned clip and setTransform
    this->DrawRobot();
    this->DrawPointCloud(); //check if nullptr!

    this->deviceContext->EndDraw(); // BLOCKS for VSync
}

void GraphicsModule::CreateBackground(Map* map) {
    std::cout << "Creating background" << std::endl;
    // Begin Context
    deviceContext->CreateCommandList(&this->commandList);
    deviceContext->SetTarget(this->commandList);
    deviceContext->BeginDraw();

    // Background Elements
    deviceContext->DrawLine(D2D1::Point2F(0, BACKGROUND_LINE_WIDTH / 2), D2D1::Point2F(CLIENT_SCREEN_WIDTH, BACKGROUND_LINE_WIDTH / 2), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(CLIENT_SCREEN_WIDTH - 2, 0), D2D1::Point2F(CLIENT_SCREEN_WIDTH - 2, CLIENT_SCREEN_HEIGHT), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(0, CLIENT_SCREEN_HEIGHT - 2), D2D1::Point2F(CLIENT_SCREEN_WIDTH, CLIENT_SCREEN_HEIGHT - 2), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(BACKGROUND_LINE_WIDTH / 2, 0), D2D1::Point2F(BACKGROUND_LINE_WIDTH / 2, CLIENT_SCREEN_HEIGHT), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);

    deviceContext->DrawLine(D2D1::Point2F(CLIENT_SCREEN_WIDTH / 2, 0), D2D1::Point2F(CLIENT_SCREEN_WIDTH / 2, CLIENT_SCREEN_HEIGHT), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(0, CLIENT_SCREEN_HEIGHT / 2), D2D1::Point2F(CLIENT_SCREEN_WIDTH, CLIENT_SCREEN_HEIGHT / 2), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);

    //draw map top right
    OLine** lines = map->GetLines();
    std::pair<float, float> temp1, temp2;

    for(int i = 0; i < map->GetLinesSize(); i++) {
        temp1 = this->XYToDipsBackground(TOP_RIGHT, lines[i]->point1->x, lines[i]->point1->y);
        temp2 = this->XYToDipsBackground(TOP_RIGHT, lines[i]->point2->x, lines[i]->point2->y);        

        deviceContext->DrawLine(
            D2D1::Point2F(temp1.first, temp1.second),
            D2D1::Point2F(temp2.first, temp2.second),
            this->brushes[COLOR_PALETTE_BLACK],
            MAP_LINE_WIDTH
        );
    }

    if(SHOW_POSSIBLE_STARTING_LOCATIONS) {
        OPoint** starts = map->GetStarts();
        for(int i = 0; i < map->GetStartsSize(); i++) {
            temp1 = this->XYToDipsBackground(TOP_RIGHT, starts[i]->x, starts[i]->y);
            deviceContext->DrawEllipse(
                D2D1::Ellipse(D2D1::Point2F(temp1.first, temp1.second), ROBOT_RADIUS, ROBOT_RADIUS),
                this->brushes[COLOR_PALETTE_BLACK],
                1.5,
                nullptr
            );
        }
    }

    // End context
    deviceContext->EndDraw();
    this->commandList->Close();
    deviceContext->SetTarget(this->targetBitmap); 
}

void GraphicsModule::UpdateRenderInfo(RenderPacket* incoming) {
    delete this->currentPacket;
    this->currentPacket = incoming;
}

void GraphicsModule::DrawRobot() {
    int quadrants[] = {BOTTOM_LEFT, TOP_RIGHT};
    for(int i = 0; i < 2; i++) {
        std::pair<float, float> temp = this->XYToDipsBackground(quadrants[i], this->currentPacket->realX, this->currentPacket->realY);
        deviceContext->DrawEllipse(
            D2D1::Ellipse(D2D1::Point2F(temp.first, temp.second), (float)ROBOT_RADIUS, (float)ROBOT_RADIUS),
            this->brushes[COLOR_PALETTE_BLACK],
            1.5,
            nullptr
        );

        deviceContext->DrawLine(
            D2D1::Point2F(temp.first, temp.second),
            D2D1::Point2F((float)(temp.first + (ROBOT_RADIUS * std::cos(this->currentPacket->realTheta))),
                          (float)(temp.second - (ROBOT_RADIUS * std::sin(this->currentPacket->realTheta)))),
            this->brushes[COLOR_PALETTE_BLACK],
            2,
            nullptr
        );
    }
}

void GraphicsModule::DrawPointCloud() {
    if(this->currentPacket->pointCloud == nullptr) {
        return;
    }

    this->deviceContext->PushAxisAlignedClip(this->sensorClip, D2D1_ANTIALIAS_MODE_PER_PRIMITIVE);

    OPoint* tempPoint;
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        if(this->currentPacket->pointCloud[i] == nullptr) {
            break;
        }

        tempPoint = this->currentPacket->pointCloud[i];
        std::pair<float, float> temp = XYToDipsBackground(BOTTOM_LEFT, tempPoint->x, tempPoint->y);
        deviceContext->FillEllipse(
            D2D1::Ellipse(D2D1::Point2F(temp.first, temp.second), LIDAR_POINT_RADIUS, LIDAR_POINT_RADIUS),
            this->brushes[COLOR_PALETTE_RED]
        );
    }

    this->deviceContext->PopAxisAlignedClip();
}

void GraphicsModule::CreateWindowModule(HINSTANCE hInstance, int nCmdShow) {
    const wchar_t CLASS_NAME[]  = L"SLAM Window Class";

    WNDCLASSW wc = { };

    wc.lpfnWndProc = ListenerModule::WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASS_NAME;

    RegisterClassW(&wc);

    RECT rect = {0, 0, CLIENT_SCREEN_WIDTH, CLIENT_SCREEN_HEIGHT};
    AdjustWindowRectEx(&rect, WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX, FALSE, 0);

    int screenWidth = GetSystemMetrics(SM_CXSCREEN);
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);

    this->hwnd = CreateWindowExW(
        0,
        CLASS_NAME,
        L"SLAM Simulator",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,

        (screenWidth - (rect.right - rect.left)) / 2, (screenHeight - (rect.bottom - rect.top)) / 2, rect.right - rect.left, rect.bottom - rect.top,

        NULL,   
        NULL,
        hInstance,
        NULL
        );

    if (hwnd == NULL) {
        std::cout << "Failed to create window handle." << std::endl;
        return;
    }

    ShowWindow(hwnd, nCmdShow);
}

void GraphicsModule::DrawStaticElements() {
    if(this->commandList) {
        this->deviceContext->DrawImage(this->commandList);
    }
}

std::pair<float, float> GraphicsModule::XYToDipsBackground(int quadrant, double x, double y) {
    switch(quadrant) {
        case TOP_LEFT:
            return {
                (CLIENT_SCREEN_WIDTH * 0.25) + (x / MM_PER_DIP) + 1,
                (CLIENT_SCREEN_HEIGHT * 0.25) + (y / MM_PER_DIP * -1) + 1
            };
        case TOP_RIGHT:
            return {
                (CLIENT_SCREEN_WIDTH * 0.75) + (x / MM_PER_DIP) - 1,
                (CLIENT_SCREEN_HEIGHT * 0.25) + (y / MM_PER_DIP * -1) + 1
            };
        case BOTTOM_LEFT:
            return {
                (CLIENT_SCREEN_WIDTH * 0.25) + (x / MM_PER_DIP) + 1,
                (CLIENT_SCREEN_HEIGHT * 0.75) + (y / MM_PER_DIP * -1) - 1
            };
        default:
            return {
                (CLIENT_SCREEN_WIDTH * 0.75) + (x / MM_PER_DIP) - 1,
                (CLIENT_SCREEN_HEIGHT * 0.75) + (y / MM_PER_DIP * -1) - 1
            };
    }
}
