#include "clickInput.h"

ClickInput::ClickInput() {

}

ClickInput::~ClickInput() {
    
}

bool ClickInput::GetClicked() {
    return this->clicked;
}

void ClickInput::SetClicked() {
    this->clicked = true;
}

void ClickInput::SetUnclicked() {
    this->clicked = false;
}

int ClickInput::GetX() {
    return this->x;
}

void ClickInput::SetX(int x) {
    this->x = x;
}

int ClickInput::GetY() {
    return this->y;
}

void ClickInput::SetY(int y) {
    this->y = y;
}
