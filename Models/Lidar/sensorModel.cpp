#include <numbers>
#include <cmath>
#include <cfloat>
#include <iostream>

#include "..\..\World\map.h"
#include "..\..\World\Objects\opoint.h"
#include "sensorModel.h"
#include "pointCloud.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\config.h"

SensorModel::SensorModel() {

}

SensorModel::~SensorModel() {
    //do not delete map here.
}

void SensorModel::GiveMap(Map* map) {
    this->map = map;
}

// for now, sensor model isn't accurate; doesn't account for acceleration/deceleration. May come back to this
// acceleration and deceleration difference turn into noise.
// with default values, that's at most 2.025 mm, and an average of 1.35 mm.
PointCloud* SensorModel::GetScan(float prevX, float prevY, float prevTheta, float currX, float currY, float currTheta) {
    PointCloud* pointCloud = new PointCloud();
    this->renderScan = new OPoint*[SENSOR_MODEL_POINTS_PER_SCAN];
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        this->renderScan[i] = nullptr;
    }
    this->currRenderPoints = 0;

    float pi = MathUtilities::PI;

    float deltaX = (currX - prevX) / SENSOR_MODEL_POINTS_PER_SCAN;
    float deltaY = (currY - prevY) / SENSOR_MODEL_POINTS_PER_SCAN;
    float deltaTheta = (currTheta - prevTheta) / SENSOR_MODEL_POINTS_PER_SCAN;
    float deltaRadian = ((2 * pi) / SENSOR_MODEL_POINTS_PER_SCAN);

    float startX = prevX + (deltaX / 2.0f);
    float startY = prevY + (deltaY / 2.0f);
    float startTheta = prevTheta + (deltaTheta / 2.0f);
    float startRadian = deltaRadian / 2.0f;

    float range;

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        range = this->GetCollisionDistance(startX, startY, startTheta + startRadian);
        pointCloud->Add(range, startRadian);

        startX += deltaX;
        startY += deltaY;
        startTheta += deltaTheta;
        startRadian += deltaRadian;
    }
    
    // pointCloud->Print();
    // this->PrintRenderCloud();

    return pointCloud;
}

OPoint** SensorModel::GetRenderScan() {
    return this->renderScan;
}

//todo ADD NOISE
float SensorModel::GetCollisionDistance(float x, float y, float theta) {
    float minDistance = FLT_MAX;
    OLine** lines = this->map->GetLines();

    float d; //discriminate
    float u; // partial derivative in respect to line length? must be 0 <= u <= 1
             // ...idk it's the percent of the line between intersection point and each end.
             // I didn't take calc 3. ¯\_(ツ)_/¯
    float t; // distance to intersection point

    float ax; // point ax
    float ay; // point ay
    float bx; // point bx. a/b can be switched with no difference
    float by; // point by

    float dx = (float)cos(theta); // direction x
    float dy = (float)sin(theta); // direction y

    for(int i = 0; i < this->map->GetLinesSize(); i++) {
        ax = lines[i]->point1->x;
        ay = lines[i]->point1->y;
        bx = lines[i]->point2->x;
        by = lines[i]->point2->y;

        d = ((bx - ax) * (-dy)) - ((by - ay) * (-dx));

        u = (((x - ax) * (-dy)) - ((y - ay) * (-dx))) / d;
        if(u < 0 || u > 1) {
            // outside the line.
            // essentially, we're seeing where a ray intersects with
            // an infinite version of our line.
            // 'u' represents whether it's within the line segment.
            continue;
        }

        t = (((bx - ax) * (y - ay)) - ((by - ay) * (x - ax))) / d;
        if(t < minDistance && t > 0) {
            // looking for closest intersection above 0 (below is behind ray)
            minDistance = t;
        }
    }

    // Based on measurement range of lidar sensor
    // Simply for rendering
    
    if(minDistance <= SENSOR_MODEL_MEASURE_MAX && minDistance >= SENSOR_MODEL_MEASURE_MIN) {
        this->renderScan[this->currRenderPoints] = new OPoint(
            x + (dx * minDistance),
            y + (dy * minDistance)
        );
        (this->currRenderPoints)++;
    }

    return minDistance;
}

void SensorModel::PrintRenderCloud() {
    std::cout << "Render cloud: " << this->currRenderPoints << std::endl;
    for(int i = 0; i < this->currRenderPoints; i++) {
        this->renderScan[i]->Print();
        std::cout << std::endl;
    }
}
