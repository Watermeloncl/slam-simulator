#ifndef SLAM_MODELS_EKF_H_
#define SLAM_MODELS_EKF_H_

#include "..\Templates\slam.h"
#include "landmarkExtractor.h"

class EKF : public SLAMModule {
public:
private:
    LandmarkExtractor* landmarkExtractor = nullptr;
    
public:
    EKF();
    ~EKF();
private:

};

#endif