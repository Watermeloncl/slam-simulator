#ifndef WORLD_MAP_H_
#define WORLD_MAP_H_

#include "Objects\oline.h"

class Map {
public:

private:
    int id = 0;

    int linesSize = -1;

    OLine** lines = nullptr;
public:
    Map(int id);
    ~Map();

    int GetLineSize();
    OLine** GetLines();

    void ReadMap(int id);
    void TestMap();
};

#endif