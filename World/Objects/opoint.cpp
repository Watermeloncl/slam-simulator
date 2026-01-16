#include <iostream>

#include "opoint.h"

OPoint::OPoint(float x, float y) {
    this->x = x;
    this->y = y;
}

OPoint::~OPoint() {

}

// No new line
void OPoint::Print() {
    std::cout << "(" << this->x << ", " << this->y << ")";
}
