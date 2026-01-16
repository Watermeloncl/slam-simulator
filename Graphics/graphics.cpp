#include <windows.h>
#include <d2d1_1.h>
#include <iostream>
#include <unordered_map>

#include "graphics.h"
#include "..\Listener\listener.h"
#include "../config.h"


GraphicsModule::GraphicsModule(HINSTANCE hInstance, int nCmdShow) {
    this->CreateWindowModule(hInstance, nCmdShow);

    this->InitD2D();
}

GraphicsModule::~GraphicsModule() {
    this->CleanupD2D();
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

    this->CreateResources();
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

    this->RenderBackground();

    this->deviceContext->EndDraw(); // BLOCKS for VSync
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

void GraphicsModule::RenderBackground() {
    if(this->commandList) {
        this->deviceContext->DrawImage(this->commandList);
    }
}

void GraphicsModule::CreateResources() {
    deviceContext->CreateCommandList(&this->commandList);
    deviceContext->SetTarget(this->commandList);
    deviceContext->BeginDraw();

    deviceContext->DrawLine(D2D1::Point2F(30, 30), D2D1::Point2F(36.4f, 30), this->brushes[COLOR_PALETTE_BLACK], 2);

    deviceContext->EndDraw();
    this->commandList->Close();
    deviceContext->SetTarget(this->targetBitmap); 
}