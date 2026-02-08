#ifndef SLAM_MODELS_GMAPPING_PARTICLE_H_
#define SLAM_MODELS_GMAPPING_PARTICLE_H_

#include "..\MapRepresentation\occupancyGrid.h"

struct Particle {
public:
    Particle();
    ~Particle();

    double oldScanX = 0.0;
    double oldScanY = 0.0;
    double oldScanTheta = 0.0;

    double currScanX = 0.0;
    double currScanY = 0.0;
    double currScanTheta = 0.0;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double nudgeX = 0.0;
    double nudgeY = 0.0;
    double nudgeTheta = 0.0;

    //used for "back update". Particles will move while slam does it's thing,
    //  and we want to take that into account, since motion isn't based on the map,
    //  we're free to assume that all moves since are valid
    //  should probably be x, y, theta
    double accumulatedPose = 0.0;

    double weight = 0.0;

    OccupancyGrid* map = nullptr;

    Particle* Copy();
    void UpdateHistory();
    void AddNudges();
};

#endif