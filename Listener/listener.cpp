#include <windows.h>
#include <windowsx.h>

#include "listener.h"
#include "clickInput.h"

MSG ListenerModule::msg = {};

ListenerModule::ListenerModule() {
    //constructor; empty
    // set private so that no instantiations can exist
}

LRESULT CALLBACK ListenerModule::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    ClickInput* clickInput = (ClickInput*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

    switch (msg) {
        case WM_LBUTTONDOWN: {
            if(clickInput == nullptr) {
                break;
            }

            clickInput->SetClicked();
            clickInput->SetX(GET_X_LPARAM(lParam));
            clickInput->SetY(GET_Y_LPARAM(lParam));

            break;
        }
        case WM_DESTROY: {
            PostQuitMessage(0);
            return 0;
        }
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}