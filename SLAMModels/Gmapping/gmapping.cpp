#include <windows.h>
#include <thread>
#include <utility>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <mutex>

#include <chrono>

#include "gmapping.h"
#include "..\Templates\slam.h"
#include "particle.h"
#include "scanMatcher.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\MapRepresentation\sector.h"
#include "..\MapRepresentation\poseRenderPacket.h"
#include "..\..\Utilities\utilities.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\config.h"

Gmapping::Gmapping() : SLAMModule() {
    this->scanMatcher = new ScanMatcher();

    this->particles = new Particle*[GMAPPING_NUM_PARTICLES];
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particles[i] = nullptr;
    }

    this->guardRenderMap = std::make_shared<std::mutex>();
}

Gmapping::~Gmapping() {
    delete this->scanMatcher;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        delete this->particles[i];
    }
    delete[] this->particles;

    //todo add way to destroy thread
}

void Gmapping::InitSlam(double startX, double startY, double startTheta) {
    this->startX = startX;
    this->startY = startY;
    this->startTheta = startTheta;

    this->slamFinished = true;
    this->slamSemaphore = CreateSemaphore(NULL, 0, 1, NULL);
    this->slamThread = std::thread(RefineEstimates, this);

    Sector* startingSector1 = new Sector();
    Sector* startingSector2 = new Sector();
    Sector* startingSector3 = new Sector();
    Sector* startingSector4 = new Sector();

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particles[i] = new Particle();
        OccupancyGrid* startingGrid = new OccupancyGrid();
        startingGrid->AddSector(0, 0, startingSector1);
        startingGrid->AddSector(-1, 0, startingSector2);
        startingGrid->AddSector(-1, -1, startingSector3);
        startingGrid->AddSector(0, -1, startingSector4);

        startingSector1->AddReference();
        startingSector2->AddReference();
        startingSector3->AddReference();
        startingSector4->AddReference();

        this->particles[i]->map = startingGrid;
        this->particles[i]->weight = 1.0 / GMAPPING_NUM_PARTICLES;
    }
}

void Gmapping::UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) {
    // what was commandtimestep for? outside history of commands

    // Check if slam algorithm updated particle poses
    // Integrate moves to new poses

    double noisyDist = 0;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        if(changeDist == 0) {
            if(changeTheta == 0) {
                break;
            }
            this->particles[i]->theta += changeTheta + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED);
            continue;
        }

        this->particles[i]->theta += (changeTheta + Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION));
        noisyDist = changeDist + Utilities::GetRandomNoise(changeDist, MOTION_MODEL_FORWARD_DEVIATION);

        this->particles[i]->x += (noisyDist * std::cos(this->particles[i]->theta));
        this->particles[i]->y += (noisyDist * std::sin(this->particles[i]->theta));
    }

    this->UpdatePoses();

    //update accumulated pose (total movement)
    //update accumulating x, y, theta within gmapping, what is the ideal? (no noise?) (movement since last scan)


    //check time stamps and update particle history if required
    if(pointCloudTimestamp != this->lastScanTimestamp) {
        this->lastScanTimestamp = pointCloudTimestamp;
        
        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            this->particles[i]->UpdateHistory();
        }

        this->lastScanExpX = this->accumulatingExpX;
        this->lastScanExpY = this->accumulatingExpY;
        this->lastScanExpTheta = this->accumulatingExpTheta;

        this->accumulatingExpX = 0.0;
        this->accumulatingExpY = 0.0;
        this->accumulatingExpTheta = 0.0;
    }

    (this->ticksSinceLastUpdate)++;

    if(pointCloud == nullptr) {
        return;
    }

    if(!(this->slamFinished)) {
        return;
    }

    if((this->ticksSinceLastUpdate >= SLAM_MAXIMUM_PERIOD_COUNT) ||
       ((this->ticksSinceLastUpdate >= SLAM_MINIMUM_PERIOD_COUNT) &&
        (this->accumulatedPoseSinceLastUpdate >= SLAM_MINIMUM_DISTANCE))) {

        this->accumulatedPoseSinceLastUpdate = 0.0;
        this->ticksSinceLastUpdate = 0;
        this->slamFinished = false;

        delete this->currPacket;
        StatePacket* packet = new StatePacket();
        packet->CopyInfo(this->particles);
        packet->pointCloud = pointCloud;

        packet->commandTimestamp = commandTimestamp;
        packet->pointCloudTimestamp = pointCloudTimestamp;

        packet->expX = this->lastScanExpX;
        packet->expY = this->lastScanExpY;
        packet->expTheta = this->lastScanExpTheta;

        this->currPacket = packet;

        ReleaseSemaphore(this->slamSemaphore, 1, NULL);
    }    
}

void Gmapping::RefineEstimates() {
    for(;;) {
        WaitForSingleObject(this->slamSemaphore, INFINITE);

        //add way to destroy
        //use this->currPacket
        //build likelihood field

        // for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {

            //add missing sectors from movement
            //nudge particles according to score
            //.........
            //checkNeff
            //  if neff below n/2, resample
            //  after resample, reset weights
        
        // }

        this->UpdateMaps();

        this->CreateRenderCopy(0);
        this->slamFinished = true;
    }
}

void Gmapping::UpdateMaps() {
    double pi = MathUtilities::PI;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);
    double inverseCellSize = 1 / GMAPPING_GRID_CELL_SIZE;
    PolarPoint** cloud = this->currPacket->pointCloud->cloud;

    // for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {

    Particle* particle = this->currPacket->particles[0]; //[i]

    std::unordered_set<std::pair<int, int>, pair_hash> misses;
    std::unordered_set<std::pair<int, int>, pair_hash> hits;

    double poseStartX = particle->oldScanX;
    double poseStartY = particle->oldScanY;
    double poseStartTheta = particle->oldScanTheta;

    double deltaX = (particle->currScanX - poseStartX) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaY = (particle->currScanY - poseStartY) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaTheta = (particle->currScanTheta - poseStartTheta) / SENSOR_MODEL_POINTS_PER_SCAN;

    double startRadian = ((pi*2) - (deltaRadian / 2));

    int cloudPoint = 0;

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; ++i) {
        if(cloud[cloudPoint]->theta == startRadian) {
            std::pair<double, double> ends = MathUtilities::PolarToCartesian(cloud[cloudPoint]->range, cloud[cloudPoint]->theta + poseStartTheta, poseStartX, poseStartY);

            int startX = (int)std::floor(poseStartX * inverseCellSize);
            int startY = (int)std::floor(poseStartY * inverseCellSize);
            int endX = (int)std::floor(ends.first * inverseCellSize);
            int endY = (int)std::floor(ends.second * inverseCellSize);

            this->AddAffectedCells(startX, startY, endX, endY, misses, hits);

            cloudPoint++;
        }

        poseStartX += deltaX;
        poseStartY += deltaY;
        poseStartTheta += deltaTheta;
        startRadian -= deltaRadian;
    }

    for(const std::pair<int, int>& hit : hits) {
        if(misses.find(hit) != misses.end()) {
            misses.erase(hit);
        }

        particle->map->ChangeCell(hit.first, hit.second, GMAPPING_LOG_ODDS_HIT);
    }

    for(const std::pair<int, int>& miss : misses) {
        particle->map->ChangeCell(miss.first, miss.second, GMAPPING_LOG_ODDS_MISS);
    }
}

void Gmapping::AddAffectedCells(int startX, int startY, int endX, int endY, std::unordered_set<std::pair<int, int>, pair_hash>& misses, std::unordered_set<std::pair<int, int>, pair_hash>& hits) {
    int dx = std::abs(endX - startX);
    int dy = std::abs(endY - startY);

    int stepX = 1;
    if(startX >= endX) {
        stepX = -1;
    }

    int stepY = 1;
    if(startY >= endY) {
        stepY = -1;
    }

    int error = dx - dy;
    int error2;

    int x = startX;
    int y = startY;

    for(;;) {
        if((x == endX) && (y == endY)) {
            hits.insert({x, y});
            break;
        }

        misses.insert({x, y});
        error2 = 2 * error;

        if(error2 > -dy) {
            error -= dy;
            x += stepX;
        }

        if(error2 < dx) {
            error += dx;
            y += stepY;
        }
    }
}

void Gmapping::CreateRenderCopy(int particleIndex) {
    std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* model = 
            this->particles[particleIndex]->map->GetCells();

    std::vector<float>* newRenderMap = new std::vector<float>();
    newRenderMap->reserve(6000);

    int sectorX, sectorY;
    double mapX, mapY, worldX, worldY;
    std::pair<float, float> screenCords;
    double cosTheta = std::cos(this->startTheta);
    double sinTheta = std::sin(this->startTheta);

    for(std::pair<std::pair<int, int>, Sector*> sectorInfo : *model) {
        sectorX = sectorInfo.first.first * GMAPPING_SECTOR_SIZE;
        sectorY = sectorInfo.first.second * GMAPPING_SECTOR_SIZE;

        for(int i = 0; i < GMAPPING_SECTOR_NUM_CELLS; i++) {
            if(sectorInfo.second->cells[i] <= 0) {
                continue;
            }

            mapX = (sectorX + (i % GMAPPING_SECTOR_SIZE)) * GMAPPING_GRID_CELL_SIZE;
            mapY = (sectorY + (i / GMAPPING_SECTOR_SIZE)) * GMAPPING_GRID_CELL_SIZE;
            worldX = (mapX*cosTheta) - (mapY*sinTheta) + this->startX;
            worldY = (mapX*sinTheta) + (mapY*cosTheta) + this->startY;

            screenCords = this->XYToDips(worldX, worldY);

            newRenderMap->push_back(screenCords.first);
            newRenderMap->push_back(screenCords.second);
            newRenderMap->push_back((float)(sectorInfo.second->cells[i]));
        }
    }

    std::lock_guard<std::mutex> lock(*(this->guardRenderMap));
    delete this->renderMap;
    this->renderMap = newRenderMap;
}

std::pair<float, float> Gmapping::XYToDips(double x, double y) {
    return {
        (CLIENT_SCREEN_WIDTH * 0.25) + (x / MM_PER_DIP) + 1,
        (CLIENT_SCREEN_HEIGHT * 0.25) + (y / MM_PER_DIP * -1) + 1
    };
}

void Gmapping::UpdatePoses() {
    std::lock_guard<std::mutex> lock(this->guardPoses);

    PoseRenderPacket* newPacket = new PoseRenderPacket(GMAPPING_NUM_PARTICLES, 3);
    int strongestIndex = this->GetStrongestParticleIndex();

    std::pair<double, double> screenCords;
    double cosTheta = std::cos(this->startTheta);
    double sinTheta = std::sin(this->startTheta);

    //rotate, translate, then convert to dips
    double worldX = ((this->particles[strongestIndex]->x)*cosTheta)-((this->particles[strongestIndex]->y)*sinTheta) + this->startX;
    double worldY = ((this->particles[strongestIndex]->x)*sinTheta)+((this->particles[strongestIndex]->y)*cosTheta) + this->startY;
    screenCords = this->XYToDips(worldX, worldY);

    newPacket->AddPose({screenCords.first, screenCords.second, this->particles[strongestIndex]->theta + this->startTheta});

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        if(i == strongestIndex) {
            continue;
        }

        worldX = ((this->particles[i]->x)*cosTheta)-((this->particles[i]->y)*sinTheta) + this->startX;
        worldY = ((this->particles[i]->x)*sinTheta)+((this->particles[i]->y)*cosTheta) + this->startY;
        screenCords = this->XYToDips(worldX, worldY);
        
        newPacket->AddPose({screenCords.first, screenCords.second, this->particles[i]->theta + this->startTheta});
    }

    this->ReplacePoses(newPacket);
}

int Gmapping::GetStrongestParticleIndex() {
    int index = 0;
    double maxWeight = -1;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        if(this->particles[i]->weight > maxWeight) {
            maxWeight = this->particles[i]->weight;
            index = i;
        }
    }

    return index;
}