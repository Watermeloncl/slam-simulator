#include <iostream>

#include "polarPoint.h"

PolarPoint::PolarPoint(float range, float theta) {
    this->range = range;
    this->theta = theta;
}

PolarPoint::~PolarPoint() {

}

// doesn't include newline.
// (r, theta)
void PolarPoint::Print() {
    std::cout << "(" << this->range << ", " << this->theta << ")";
}