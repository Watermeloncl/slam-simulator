#include <unordered_map>
#include <iostream>
#include <cmath>
#include <vector>

#include "occupancyGrid.h"
#include "sector.h"

OccupancyGrid::OccupancyGrid() {

}

OccupancyGrid::~OccupancyGrid() {
    //TODO what goes here?
}

void OccupancyGrid::AddSector(int x, int y, Sector* sector) {
    this->grid[{x, y}] = sector;
}

void OccupancyGrid::RemoveSector(int x, int y) {
    this->grid.erase({x, y});
}

bool OccupancyGrid::SectorExists(int x, int y) {
    if(this->grid.find({x, y}) != this->grid.end()) {
        return true;
    } else {
        return false;
    }
}

//returns nullptr if it doesn't exist
Sector* OccupancyGrid::GetSector(int x, int y) {
    if(this->grid.find({x, y}) != this->grid.end()) {
        return this->grid[{x, y}];
    } else {
        return nullptr;
    }
}

void OccupancyGrid::GetRowWalls(std::vector<double>& values, int sectorX, int sectorY, int row) {
    values.clear();

    if(this->grid.find({sectorX, sectorY}) == this->grid.end()) {
        for(int i = 0; i < GMAPPING_SECTOR_SIZE; i++) {
            values.push_back(GMAPPING_INF);
        }
        return;
    }

    Sector* thisSector = this->grid[{sectorX, sectorY}];

    int cellIndex = GMAPPING_SECTOR_SIZE * row;
    for(int i = 0; i < GMAPPING_SECTOR_SIZE; i++) {
        if(thisSector->cells[cellIndex] >= GMAPPING_LOG_ODDS_WALL_VALUE) {
            values.push_back(0);
        } else {
            values.push_back(GMAPPING_INF);
        }

        cellIndex++;
    }
}

//either log odds hit or miss
void OccupancyGrid::ChangeCell(int cellX, int cellY, double value) {
    int sectorX = (int)std::floor((double)cellX / (double)GMAPPING_SECTOR_SIZE);
    int sectorY = (int)std::floor((double)cellY / (double)GMAPPING_SECTOR_SIZE);
    std::unordered_map<std::pair<int, int>, Sector*>::iterator it = this->grid.find({sectorX, sectorY});

    if(it == this->grid.end()) {
        Sector* newSector = new Sector();
        this->AddSector(sectorX, sectorY, newSector);
        newSector->AddReference();
        newSector->ChangeCellValue(cellX % GMAPPING_SECTOR_SIZE, cellY % GMAPPING_SECTOR_SIZE, value);

        return;
    }

    if(it->second->GetReferenceCount() == 1) {
        it->second->ChangeCellValue(cellX % GMAPPING_SECTOR_SIZE, cellY % GMAPPING_SECTOR_SIZE, value);
    } else {
        Sector* oldSector = it->second;
        Sector* newSector = oldSector->Copy();
        newSector->AddReference();
        newSector->ChangeCellValue(cellX % GMAPPING_SECTOR_SIZE, cellY % GMAPPING_SECTOR_SIZE, value);
        this->RemoveSector(sectorX, sectorY);
        oldSector->RemoveReference();
    }
}

std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* OccupancyGrid::GetCells() {
    return &(this->grid);
}