#include <unordered_map>
#include <iostream>
#include <cmath>

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