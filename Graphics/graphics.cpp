#include <windows.h>
#include <d2d1.h>
#include <iostream>

#include "../config.h"
#include "graphics.h"
#include "..\Listener\listener.h"


GraphicsModule::GraphicsModule(HINSTANCE hInstance, int nCmdShow) {
    this->CreateWindowModule(hInstance, nCmdShow);

    this->InitD2D();
}

GraphicsModule::~GraphicsModule() {
    this->CleanupD2D();
}

void GraphicsModule::InitD2D() {
    D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &(this->factory));

    RECT rc;
    GetClientRect(this->hwnd, &rc);

    D2D1_RENDER_TARGET_PROPERTIES props = 
        D2D1::RenderTargetProperties(D2D1_RENDER_TARGET_TYPE_DEFAULT,
                                     D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM,
                                                       D2D1_ALPHA_MODE_IGNORE));

    D2D1_HWND_RENDER_TARGET_PROPERTIES hwndProps =
        D2D1::HwndRenderTargetProperties(this->hwnd,
                                         D2D1::SizeU(rc.right - rc.left, rc.bottom - rc.top),
                                         D2D1_PRESENT_OPTIONS_NONE); // VSync enabled

    factory->CreateHwndRenderTarget(&props, &hwndProps, &(this->renderTarget));
    this->renderTarget->CreateSolidColorBrush(D2D1::ColorF(0xFF0000), &(this->brush));
}

void GraphicsModule::CleanupD2D() {
    if (this->brush) brush->Release();
    // if (bitmap) bitmap->Release();
    if (this->renderTarget) renderTarget->Release();
    if (factory) factory->Release();
}

void GraphicsModule::RenderFrame(UINT32* pixels) {
    this->renderTarget->BeginDraw();
    this->renderTarget->Clear(D2D1::ColorF(0xe3e3e3));

    D2D1_ELLIPSE ellipse = D2D1::Ellipse(D2D1::Point2F(250, 350), 25, 45);
    renderTarget->FillEllipse(ellipse, this->brush);

    this->renderTarget->EndDraw(); // BLOCKS for VSync
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