#ifndef SLAM_MODELS_GMAPPING_H_
#define SLAM_MODELS_GMAPPING_H_

#include "..\Templates\slam.h"
#include "particle.h"
#include "scanMatcher.h"

class Gmapping : public SLAMModule {
public:
private:
    ScanMatcher* scanMatcher = nullptr;

public:
    Gmapping();
    ~Gmapping();
private:
};

#endif