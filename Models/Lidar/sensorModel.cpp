#include <numbers>
#include <cmath>
#include <cfloat>
#include <iostream>

#include "..\..\World\map.h"
#include "..\..\World\Objects\opoint.h"
#include "sensorModel.h"
#include "pointCloud.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\Utilities\utilities.h"
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
PointCloud* SensorModel::GetScan(double prevX, double prevY, double prevTheta, double currX, double currY, double currTheta) {
    PointCloud* pointCloud = new PointCloud();
    this->renderScan = new OPoint*[SENSOR_MODEL_POINTS_PER_SCAN];
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        this->renderScan[i] = nullptr;
    }
    this->currRenderPoints = 0;

    double pi = MathUtilities::PI;

    double deltaX = (currX - prevX) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaY = (currY - prevY) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaTheta = (currTheta - prevTheta) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaRadian = ((2 * pi) / SENSOR_MODEL_POINTS_PER_SCAN);

    double startX = prevX + (deltaX / 2.0);
    double startY = prevY + (deltaY / 2.0);
    double startTheta = prevTheta + (deltaTheta / 2.0);
    double startRadian = deltaRadian / 2.0;

    double range, dx, dy, resolution;

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        range = this->GetCollisionDistance(startX, startY, startTheta + startRadian, dx, dy);
        if(range < SENSOR_MODEL_MEASURE_MIN || range > SENSOR_MODEL_MEASURE_MAX) {
            // out of range
            continue;
        }

        for(int tier = 0; tier < SENSOR_MODEL_ACCURACY_TIERS; tier++) {
            if(range < SENSOR_MODEL_ACCURACY[tier].second) {
                range = Utilities::AddRangeToNoise(range, tier);
                break;
            }
        }

        resolution = range * SENSOR_MODEL_RANGE_RESOLUTION;
        range = std::round(range / resolution) * resolution;

        pointCloud->Add(range, startRadian);

        // For rendering purposes
        this->renderScan[this->currRenderPoints] = new OPoint(
            startX + (dx * range),
            startY + (dy * range)
        );
        (this->currRenderPoints)++;

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
double SensorModel::GetCollisionDistance(double x, double y, double theta, double& dx, double& dy) {
    double minDistance = FLT_MAX;
    OLine** lines = this->map->GetLines();

    double d; //discriminate
    double u; // partial derivative in respect to line length? must be 0 <= u <= 1
             // ...idk it's the percent of the line between intersection point and each end.
             // I didn't take calc 3. ¯\_(ツ)_/¯
    double t; // distance to intersection point

    double ax; // point ax
    double ay; // point ay
    double bx; // point bx. a/b can be switched with no difference
    double by; // point by

    dx = cos(theta); // direction x
    dy = sin(theta); // direction y

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

    return minDistance;
}

void SensorModel::PrintRenderCloud() {
    std::cout << "Render cloud: " << this->currRenderPoints << std::endl;
    for(int i = 0; i < this->currRenderPoints; i++) {
        this->renderScan[i]->Print();
        std::cout << std::endl;
    }
}
