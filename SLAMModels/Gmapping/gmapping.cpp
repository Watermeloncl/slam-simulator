#include <windows.h>
#include <thread>
#include <utility>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <cfloat>
#include <chrono>

#include "gmapping.h"
#include "logField.h"
#include "..\Templates\slam.h"
#include "particle.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\MapRepresentation\sector.h"
#include "..\MapRepresentation\poseRenderPacket.h"
#include "..\..\World\Objects\opoint.h"
#include "..\..\Utilities\utilities.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\config.h"

Gmapping::Gmapping() : SLAMModule() {
    this->particles = new Particle*[GMAPPING_NUM_PARTICLES];
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particles[i] = nullptr;
    }

    this->guardRenderMap = std::make_shared<std::mutex>();

    this->history = new std::pair<double, double>[GMAPPING_HISTORY_SIZE];
}

Gmapping::~Gmapping() {
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
    this->refreshParticles = false;

    this->particlesToRefresh = new int[GMAPPING_NUM_PARTICLES];
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particlesToRefresh[i] = i;
    }

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
        this->particles[i]->weight = GMAPPING_STARTING_WEIGHT;
    }
}

void Gmapping::UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) {
    // what was commandtimestep for? outside history of commands

    // Check if slam algorithm updated particle poses
    // Integrate moves to new poses

    if(!(this->slamFinished)) {
        //track history
        this->history[this->historySize].first = changeDist;
        this->history[this->historySize].second = changeTheta;
        (this->historySize)++;
    } else if(this->slamFinished && !(this->backUpdated)) {
        this->backUpdated = true;

        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            this->particles[i]->x = this->currPacket->particles[i]->x;
            this->particles[i]->y = this->currPacket->particles[i]->y;
            this->particles[i]->theta = this->currPacket->particles[i]->theta;
            this->particles[i]->weight = this->currPacket->particles[i]->weight;
        }

        if(this->refreshParticles) {
            this->FlushRefreshedParticles();
        }

        for(int i = 0; i < this->historySize; i++) {
            this->MoveParticles(this->history[i].first, this->history[i].second);
        }
        this->historySize = 0;
    }

    this->MoveParticles(changeDist, changeTheta);
    this->UpdatePoses();

    this->accumulatingExpDist += changeDist;
    this->accumulatingExpTheta += changeTheta;

    this->accumulatedPoseSinceLastUpdate += std::abs(changeDist);
    this->accumulatedPoseSinceLastUpdate += (std::abs(changeTheta) * MOTION_MODEL_ROTATION_AMP);

    //check time stamps and update particle history if required
    if(pointCloudTimestamp != this->lastScanTimestamp) {
        this->lastScanTimestamp = pointCloudTimestamp;
        
        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            this->particles[i]->UpdateHistory();
        }

        this->lastScanExpDist = this->accumulatingExpDist;
        this->lastScanExpTheta = this->accumulatingExpTheta;

        this->accumulatingExpDist = 0.0;
        this->accumulatingExpTheta = 0.0;
    }

    (this->ticksSinceLastUpdate)++;

    if((pointCloud == nullptr) || (pointCloud->cloudSize == 0)) {
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
        this->backUpdated = false;

        delete this->currPacket;
        StatePacket* packet = new StatePacket();
        packet->CopyInfo(this->particles);
        packet->pointCloud = pointCloud;

        packet->commandTimestamp = commandTimestamp;
        packet->pointCloudTimestamp = pointCloudTimestamp;

        packet->expDist = this->lastScanExpDist;
        packet->expTheta = this->lastScanExpTheta;

        this->currPacket = packet;

        ReleaseSemaphore(this->slamSemaphore, 1, NULL);
    }    
}

void Gmapping::MoveParticles(double changeDist, double changeTheta) {
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
}

void Gmapping::RefineEstimates() {
    for(;;) {
        WaitForSingleObject(this->slamSemaphore, INFINITE);

        //add way to destroy
        //use this->currPacket

        // vvvv steps 3-7 vvvv

        LogField* logField = new LogField();
        logField->cloud = this->currPacket->pointCloud->cloud;
        logField->numPoints = this->currPacket->pointCloud->cloudSize;
        this->CreateInverseSigmas(logField);

        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            this->GetPoints(logField, i);
            logField->currParticle = this->currPacket->particles[i];

            //num points will not be 0.
            this->GetSectorRange(logField);
            logField->likelihoodField.reserve(((logField->sectorMaxX)-(logField->sectorMinX)) * ((logField->sectorMaxY)-(logField->sectorMinY)) * (GMAPPING_SECTOR_SIZE*GMAPPING_SECTOR_SIZE));

            this->CreateGrid(logField);
            this->PopulateLikelihoodField(logField);

            this->NudgeParticle(logField);

            //check local samples and build L(k)
            this->SampleParticles(logField);

            delete logField->cartesianPoints;
        }

        delete logField;
        // ^^^^ steps 3-7 ^^^^

        // vvvv step 8 (-9) vvvv
        double totalWeight = 0.0;
        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            totalWeight += this->currPacket->particles[i]->weight;
        }

        double neffWeight = 0.0;
        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            this->currPacket->particles[i]->weight /= totalWeight;
            neffWeight += (this->currPacket->particles[i]->weight)*(this->currPacket->particles[i]->weight);
        }

        // particle efficiency; Neff = 1 / ( sum(final_weights^2))
        double neff = 1.0 / neffWeight;
        // std::cout << "neff: " << neff << std::endl;
        // ^^^^ step 8 (-9) ^^^^

        // steps 12, 13
        this->UpdateMaps();
        this->CreateRenderCopy();

        // steps 9-11
        if(neff < (GMAPPING_NEFF_THRESHOLD)) {
            this->FillParticlesToRefresh();
        }

        this->slamFinished = true;
    }
}

void Gmapping::UpdateMaps() {
    double pi = MathUtilities::PI;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);
    double inverseCellSize = 1 / GMAPPING_GRID_CELL_SIZE;
    if(this->currPacket->pointCloud == nullptr) {
        return;
    }

    PolarPoint** cloud = this->currPacket->pointCloud->cloud;
    if(this->currPacket->pointCloud->cloudSize == 0) {
        return;
    }

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        Particle* particle = this->currPacket->particles[i];
        if(particle->map == nullptr) {
            continue;
        }

        //todo combine with getPoints()?
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
        int totalPoints = this->currPacket->pointCloud->cloudSize;

        for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; ++i) {
            if(cloud[cloudPoint]->theta == startRadian) {
                std::pair<double, double> ends = MathUtilities::PolarToCartesian(cloud[cloudPoint]->range, cloud[cloudPoint]->theta + poseStartTheta, poseStartX, poseStartY);

                int startX = (int)std::floor(poseStartX * inverseCellSize);
                int startY = (int)std::floor(poseStartY * inverseCellSize);
                int endX = (int)std::floor(ends.first * inverseCellSize);
                int endY = (int)std::floor(ends.second * inverseCellSize);

                this->AddAffectedCells(startX, startY, endX, endY, misses, hits);

                cloudPoint++;
                if(cloudPoint == totalPoints) {
                    break;
                }
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

void Gmapping::CreateRenderCopy() {
    int strongestIndex = this->GetStrongestParticleIndex(this->currPacket->particles);
    std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* model = 
            this->currPacket->particles[strongestIndex]->map->GetCells();

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
    int strongestIndex = this->GetStrongestParticleIndex(this->particles);

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

int Gmapping::GetStrongestParticleIndex(Particle** particles) {
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

void Gmapping::CreateInverseSigmas(LogField* logField) {
    double* newSigmas = new double[logField->numPoints]();
    std::fill_n(newSigmas, logField->numPoints, GMAPPING_SCAN_MATCHING_DEFAULT_SIGMA);

    double sigmaMM;
    for(int i = 0; i < logField->numPoints; i++) {
        for(int tier = 0; tier < SENSOR_MODEL_ACCURACY_TIERS; tier++) {
            if(logField->cloud[i]->range < SENSOR_MODEL_ACCURACY[tier].second) {
                sigmaMM = (std::max)(SENSOR_MODEL_ACCURACY[tier].first * logField->cloud[i]->range, GMAPPING_GRID_CELL_SIZE / 2.0);
                newSigmas[i] = -1.0 / (2.0*sigmaMM*sigmaMM);
                break;
            }
        }
    }

    logField->inverseSigmas = newSigmas;
}

void Gmapping::GetPoints(LogField* logField, int particleIndex) {
    logField->currParticle = this->currPacket->particles[particleIndex];
    Particle* particle = logField->currParticle;
    PolarPoint** cloud = this->currPacket->pointCloud->cloud;

    if(logField->numPoints == 0) {
        return;
    }

    double pi = MathUtilities::PI;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);

    double poseStartX = particle->oldScanX;
    double poseStartY = particle->oldScanY;
    double poseStartTheta = particle->oldScanTheta;

    double deltaX = (particle->currScanX - poseStartX) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaY = (particle->currScanY - poseStartY) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaTheta = (particle->currScanTheta - poseStartTheta) / SENSOR_MODEL_POINTS_PER_SCAN;

    double startRadian = ((pi*2) - (deltaRadian / 2));

    int cloudPoint = 0;


    OPoint** returnPoints = new OPoint*[logField->numPoints];

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; ++i) {
        if(cloud[cloudPoint]->theta == startRadian) {
            std::pair<double, double> ends = MathUtilities::PolarToCartesian(cloud[cloudPoint]->range, cloud[cloudPoint]->theta + poseStartTheta, poseStartX, poseStartY);

            returnPoints[cloudPoint] = new OPoint(ends.first, ends.second);

            cloudPoint++;
            if(cloudPoint == logField->numPoints) {
                break;
            }
        }

        poseStartX += deltaX;
        poseStartY += deltaY;
        poseStartTheta += deltaTheta;
        startRadian -= deltaRadian;
    }

    logField->cartesianPoints = returnPoints;
}

// include safety of +- 1 both x/y
void Gmapping::GetSectorRange(LogField* logField) {
    double minX = DBL_MAX;
    double maxX = -DBL_MAX;
    double minY = DBL_MAX;
    double maxY = -DBL_MAX;

    if(logField->cartesianPoints == nullptr) {
        return;
    }

    for(int i = 0; i < this->currPacket->pointCloud->cloudSize; i++) {
        minX = (std::min)(minX, logField->cartesianPoints[i]->x);
        maxX = (std::max)(maxX, logField->cartesianPoints[i]->x);
        minY = (std::min)(minY, logField->cartesianPoints[i]->y);
        maxY = (std::max)(maxY, logField->cartesianPoints[i]->y);
    }

    double inverseCellSize = 1.0 / GMAPPING_GRID_CELL_SIZE;
    logField->sectorMinX = ((int)std::floor(((double)std::floor(minX * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) - 1;
    logField->sectorMinY = ((int)std::floor(((double)std::floor(minY * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) - 1;
    logField->sectorMaxX = ((int)std::floor(((double)std::floor(maxX * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) + 1;
    logField->sectorMaxY = ((int)std::floor(((double)std::floor(maxY * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) + 1;

    logField->offsetX = logField->sectorMinX * -GMAPPING_SECTOR_MM_SIZE;
    logField->offsetY = logField->sectorMinY * -GMAPPING_SECTOR_MM_SIZE;
}

void Gmapping::CreateGrid(LogField* logField) {
    logField->sectorsWidth = logField->sectorMaxX - logField->sectorMinX + 1;
    logField->sectorsHeight = logField->sectorMaxY - logField->sectorMinY + 1;
    int totalCells = logField->sectorsWidth * logField->sectorsHeight * GMAPPING_SECTOR_SIZE * GMAPPING_SECTOR_SIZE;
    logField->likelihoodField.assign(totalCells, GMAPPING_INF);

    int lfIndex = 0;
    int cellIndex;

    for(int y = logField->sectorMinY; y <= logField->sectorMaxY; ++y) {
        std::vector<Sector*> rowSectors;
        rowSectors.reserve(logField->sectorsWidth);

        for(int x = logField->sectorMinX; x <= logField->sectorMaxX; ++x) {
            rowSectors.push_back(logField->currParticle->map->GetSector(x, y));
        }

        for(int row = 0; row < GMAPPING_SECTOR_SIZE; row++) {
            for(Sector* currSector : rowSectors) {
                if(currSector == nullptr) {
                    lfIndex += GMAPPING_SECTOR_SIZE;
                } else {
                    cellIndex = row * GMAPPING_SECTOR_SIZE;
                    for(int i = 0; i < GMAPPING_SECTOR_SIZE; i++) {
                        if(currSector->cells[cellIndex] >= GMAPPING_LOG_ODDS_WALL_VALUE) {
                            logField->likelihoodField[lfIndex] = 0;
                        }
                        cellIndex++;
                        lfIndex++;
                    }
                }
            }
        }
    }
}

// Euclidean Distance Transform using Felzenszwalb-Huttenlocher (FH) algorithm
// Do 1D distance transform first on rows, then on cols. Expects a grid of all INF except obstacle cells, which have 0
void Gmapping::PopulateLikelihoodField(LogField* logField) {
    // truncate distances?
    logField->cellsWidth = logField->sectorsWidth * GMAPPING_SECTOR_SIZE;
    logField->cellsHeight = logField->sectorsHeight * GMAPPING_SECTOR_SIZE;
    int width = logField->cellsWidth;
    int height = logField->cellsHeight;

    // Pass 1: Rows
    std::vector<double> row(width);
    std::vector<double> resultRows(width);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            row[x] = logField->likelihoodField[(y * width) + x];
        }
        
        this->DistanceTransform1D(row, resultRows, width);
        
        for (int x = 0; x < width; ++x) {
            logField->likelihoodField[(y * width) + x] = resultRows[x];
        }
    }

    double cellSizeSquared = GMAPPING_GRID_CELL_SIZE*GMAPPING_GRID_CELL_SIZE;

    // Pass 2: Columns
    std::vector<double> col(height);
    std::vector<double> resultCols(height);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            col[y] = logField->likelihoodField[(y * width) + x];
        }
        
        this->DistanceTransform1D(col, resultCols, height);
        
        for (int y = 0; y < height; ++y) {
            logField->likelihoodField[(y * width) + x] = (std::min)(resultCols[y] * cellSizeSquared, GMAPPING_SCAN_MATCHING_MAX_DIST);
        }
    }
}

// Get's the lower envelope of a set of parabolas, each representing the closeness of that index's obstacle
// inDists represents distances gained during previous pass
// result is output
// n is size of arrays
void Gmapping::DistanceTransform1D(const std::vector<double>& inDists, std::vector<double>& result, int n) {
    std::vector<int> v(n);      // locations of parabolas in lower envelope
    std::vector<double> z(n + 1); // locations of boundaries between parabolas, one extra for upper bound
    
    int k = 0; // parabola num

    v[0] = 0; // init 
    z[0] = -GMAPPING_INF; // init starting lower envelope
    z[1] = +GMAPPING_INF;

    // intersection of two parabolas, s = (f(q) + q^2 - (f(p) + p^2)) / (2q - 2p)
    for (int q = 1; q < n; ++q) {
        // Calculate intersection of parabola q and v[k]
        double s = ((inDists[q] + q * q) - (inDists[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
        
        while (s <= z[k]) {
            k--; // "pop off" occluded parabolas, or parabolas not involved in lower envelope
            s = ((inDists[q] + q*q) - (inDists[v[k]] + (v[k]*v[k]))) / ((2.0*q) - (2.0*v[k]));
        }
        
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = +GMAPPING_INF;
    }

    // Fill in the distance values from the lower envelope
    int j = 0;
    for (int q = 0; q < n; ++q) {
        while (z[j + 1] < q) {
            j++;
        }
        result[q] = ((q - v[j])*(q - v[j])) + inDists[v[j]];
    }
}

void Gmapping::NudgeParticle(LogField* logField) {

    // original, +x, -x, +y, -y, +t, -t
    std::vector<double> scores(7);
    for(int round = 0; round < GMAPPING_SCAN_MATCHING_MAX_NUDGES; round++) {
        scores[0] = this->ScoreRelativePosition(logField, 0, 0, 0);
        scores[1] = this->ScoreRelativePosition(logField, GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0, 0);
        scores[2] = this->ScoreRelativePosition(logField, -GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0, 0);
        scores[3] = this->ScoreRelativePosition(logField, 0, GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0);
        scores[4] = this->ScoreRelativePosition(logField, 0, -GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0);
        scores[5] = this->ScoreRelativePosition(logField, 0, 0, GMAPPING_SCAN_MATCHING_NUDGE_TWIST);
        scores[6] = this->ScoreRelativePosition(logField, 0, 0, -GMAPPING_SCAN_MATCHING_NUDGE_TWIST);

        int index = 0;
        double currMax = scores[0];
        for(int i = 1; i < 7; i++) {
            if(scores[i] > currMax) {
                currMax = scores[i];
                index = i;
            }
        }

        if(index == 0) {
            break;
        }

        switch(index) {
            case 0:
                break;
            case 1:
                logField->currParticle->nudgeX += GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 2:
                logField->currParticle->nudgeX -= GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 3:
                logField->currParticle->nudgeY += GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 4:
                logField->currParticle->nudgeY -= GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 5:
                logField->currParticle->nudgeTheta += GMAPPING_SCAN_MATCHING_NUDGE_TWIST;
                break;
            default:
                logField->currParticle->nudgeTheta -= GMAPPING_SCAN_MATCHING_NUDGE_TWIST;
                break;
        }
    }
}

void Gmapping::SampleParticles(LogField* logField) {
    //neighbors to sample
    double tTerms[3] = {0, GMAPPING_SCAN_MATCHING_NUDGE_JUMP, -GMAPPING_SCAN_MATCHING_NUDGE_JUMP};
    double rTerms[3] = {0, GMAPPING_SCAN_MATCHING_NUDGE_TWIST, -GMAPPING_SCAN_MATCHING_NUDGE_TWIST};

    //calc P(Xt | Xt-1, u)
    //calc P(Zt | Xt, m)
    double maxPower = -DBL_MAX;
    std::vector<double> motionScore(27);
    std::vector<double> scanScore(27);
    int index;
    for(int i = 0; i < 3; i++) { // theta
        for(int j = 0; j < 3; j++) { // y
            for(int k = 0; k < 3; k++) { // x
                index = (i * 9) + (j * 3) + k;
                motionScore[index] = this->ScoreParticlePose(logField, tTerms[k], tTerms[j], rTerms[i]);
                scanScore[index] = this->ScoreRelativePosition(logField, tTerms[k], tTerms[j], rTerms[i]);
                maxPower = (std::max)(maxPower, scanScore[index]);
            }
        }
    }

    //sorta normalize with maxPower; brings probabilities into calculatable range
    for(int i = 0; i < 27; i++) {
        scanScore[i] -= maxPower;
    }

    double startTheta = logField->currParticle->theta + logField->currParticle->nudgeTheta;
    
    double totalWeight = 0.0;
    double sampleWeight = 0.0;
    double totalX = 0.0;
    double totalY = 0.0;
    double totalSinTheta = 0.0;
    double totalCosTheta = 0.0;

    // std::cout << "================================== p scores ==============================" << std::endl;

    for(int i = 0; i < 3; i++) { // theta
        for(int j = 0; j < 3; j++) { // y
            for(int k = 0; k < 3; k++) { // x
                index = (i * 9) + (j * 3) + k;

                scanScore[index] = std::exp(scanScore[index]);
                motionScore[index] = std::exp(motionScore[index]);

                // std::cout << scanScore[index] << " " << motionScore[index] << std::endl;

                sampleWeight = scanScore[index]*motionScore[index];
                totalWeight += sampleWeight;

                totalX += sampleWeight * (logField->currParticle->x + logField->currParticle->nudgeX + tTerms[k]);
                totalY += sampleWeight * (logField->currParticle->y + logField->currParticle->nudgeY + tTerms[j]);

                totalSinTheta += sampleWeight * std::sin(startTheta + rTerms[i]);
                totalCosTheta += sampleWeight * std::cos(startTheta + rTerms[i]);
            }
        }
    }

    if(totalWeight > 0) {
        logField->currParticle->x = totalX / totalWeight;
        logField->currParticle->y = totalY / totalWeight;
        logField->currParticle->theta = std::atan2(totalSinTheta, totalCosTheta);

        logField->currParticle->weight *= totalWeight;
    }
}

//TODO might need to change motion_model_forward_deviation
//TODO doesn't work for turning yet (gets nan)
double Gmapping::ScoreParticlePose(LogField* logField, double deltaX, double deltaY, double deltaTheta) {
    // based on mahalanobis distance: e^-(diff^2 / 2*sigma^2)
    double xDiff = logField->currParticle->currScanX - logField->currParticle->oldScanX + deltaX + logField->currParticle->nudgeX;
    double yDiff = logField->currParticle->currScanY - logField->currParticle->oldScanY + deltaY + logField->currParticle->nudgeY;
    double deltaDist = std::sqrt(xDiff*xDiff + yDiff*yDiff);

    double distDiff = deltaDist - this->currPacket->expDist;

    double thetaTravelled = logField->currParticle->currScanTheta - logField->currParticle->oldScanTheta + deltaTheta + logField->currParticle->nudgeTheta;
    double thetaDiff = thetaTravelled - (this->currPacket->expTheta);
    double thetaError = std::atan2(std::sin(thetaDiff), std::cos(thetaDiff));

    double distSigma = deltaDist * MOTION_MODEL_FORWARD_DEVIATION + MOTION_MODEL_BASE_FORWARD_DEVIATION;

    double sigmaRotation = (thetaTravelled * MOTION_MODEL_ROTATION);
    double sigmaVeer = (deltaDist * MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
    double thetaSigma = std::sqrt(sigmaRotation*sigmaRotation + sigmaVeer*sigmaVeer + MOTION_MODEL_BASE_ROTATION_DEVIATION*MOTION_MODEL_BASE_ROTATION_DEVIATION);

    return -0.5 * (((distDiff*distDiff) / (distSigma*distSigma)) + ((thetaError*thetaError) / (thetaSigma*thetaSigma)));
}

//todo: how to deal with a point if it goes off the grid? should we worry about that?
double Gmapping::ScoreRelativePosition(LogField* logField, double deltaX, double deltaY, double deltaTheta) {
    //given a set of points and a set of changes,
    //   alter all the points in the cloud accordingly and count up the score

    // based on mahalanobis distance: e^-(d^2 / (2*sigma^2))
    // the log of that can be added together to get the same idea. Just looking for direction

    double score = 0;
    double cosTheta = std::cos(deltaTheta + logField->currParticle->nudgeTheta);
    double sinTheta = std::sin(deltaTheta + logField->currParticle->nudgeTheta);
    double x, y, tempX;
    int invScanPercent = (int)(1.0 / GMAPPING_SCAN_MATCHING_PERCENT_LASERS_USED);
    double invCellSize = 1.0 / GMAPPING_GRID_CELL_SIZE;
    for(int i = 0; i < logField->numPoints; i += invScanPercent) {

        //add nudges?
        x = (logField->cartesianPoints[i]->x) + deltaX + logField->currParticle->nudgeX;
        y = (logField->cartesianPoints[i]->y) + deltaY + logField->currParticle->nudgeY;

        if(deltaTheta) {
            tempX = x;
            x = tempX*cosTheta - y*sinTheta;
            y = tempX*sinTheta + y*cosTheta;
        }

        //convert to likelihood field from cartesian world
        x += logField->offsetX;
        y += logField->offsetY;

        score += logField->likelihoodField[(int)((std::floor(y * invCellSize)*(logField->cellsWidth)) + std::floor(x * invCellSize))] * logField->inverseSigmas[i];
    }

    return score;
}

// Choose with replacement (uses comb randomness; guarentees the best particles get chosen at least once)
void Gmapping::FillParticlesToRefresh() {

    // Pick particles for refresh
    double offset = 1.0 / GMAPPING_NUM_PARTICLES;
    double position = Utilities::GetUniformEpsilon(offset);

    Particle** particles = this->currPacket->particles;
    double currWeightMass = particles[0]->weight;
    int particleNum = 0;

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        while(currWeightMass <= position) {
            particleNum++;
            currWeightMass += particles[particleNum]->weight;
        }

        this->particlesToRefresh[i] = particleNum;
        position += offset;
    }

    // Determine particle assignment
    std::vector<int> unassigned; //particles not yet assigned a location
    std::vector<int> unfulfilled; //simply nums 0-numparticles that aren't in particlesToRefresh
    unassigned.reserve(8);
    unfulfilled.reserve(8);

    int checkPosition = 0;

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        while(i > this->particlesToRefresh[checkPosition]) {
            unassigned.push_back(checkPosition);
            checkPosition++;
            if(checkPosition == GMAPPING_NUM_PARTICLES) {
                break;
            }
        }

        //edge case
        if(checkPosition == GMAPPING_NUM_PARTICLES) {
            while(i < GMAPPING_NUM_PARTICLES) {
                unfulfilled.push_back(i);
                i++;
            }
            break;
        }

        if(i < this->particlesToRefresh[checkPosition]) {
            unfulfilled.push_back(i);
        } else if(i == this->particlesToRefresh[checkPosition]) {
            checkPosition++;
        }
    }

    //edge case
    while(checkPosition < GMAPPING_NUM_PARTICLES) {
        unassigned.push_back(checkPosition);
        checkPosition++;
    }

    // Run through the other particles and clear their occupancy maps, as well as
    //  add the new one in. Just need to add maps; later, when we refresh, we
    //  change the other values
    for(int i = 0; i < ((int)(unfulfilled.size())); i++) {
        this->currPacket->particles[unfulfilled[i]]->map->ClearSectors();
        this->currPacket->particles[unfulfilled[i]]->map->FillSectors(this->currPacket->particles[this->particlesToRefresh[unassigned[i]]]->map->GetSectors());
    }

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->currPacket->particles[i]->weight = GMAPPING_STARTING_WEIGHT;
    }

    this->refreshParticles = true;
}

void Gmapping::FlushRefreshedParticles() {
    Particle* sourceParticle;
    Particle* destinationParticle;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        destinationParticle = this->particles[i];
        sourceParticle = this->particles[this->particlesToRefresh[i]];

        destinationParticle->oldScanX = sourceParticle->oldScanX;
        destinationParticle->oldScanY = sourceParticle->oldScanY;
        destinationParticle->oldScanTheta = sourceParticle->oldScanTheta;
        
        destinationParticle->currScanX = sourceParticle->currScanX;
        destinationParticle->currScanY = sourceParticle->currScanY;
        destinationParticle->currScanTheta = sourceParticle->currScanTheta;

        destinationParticle->x = sourceParticle->x;
        destinationParticle->y = sourceParticle->y;
        destinationParticle->theta = sourceParticle->theta;

        destinationParticle->accumulatedPose = sourceParticle->accumulatedPose;
    }
}
