#ifndef WORLD_OBJECTS_OPOINT_H_
#define WORLD_OBJECTS_OPOINT_H_

struct OPoint {
public:
    OPoint(float x, float y);
    ~OPoint();

    float x;
    float y;

    void Print();
};

#endif