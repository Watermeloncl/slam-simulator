#include <windows.h>
#include <windowsx.h>
#include "listener.h"

#include "listener.h"

MSG ListenerModule::msg = {};

ListenerModule::ListenerModule() {
    //constructor; empty
    // set private so that no instantiations can exist
}

LRESULT CALLBACK ListenerModule::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_DESTROY: {
            PostQuitMessage(0);
            return 0;
            break;
        }        
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}