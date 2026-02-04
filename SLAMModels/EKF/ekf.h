#ifndef SLAM_MODELS_EKF_H_
#define SLAM_MODELS_EKF_H_

#include "..\Templates\slam.h"
#include "landmarkExtractor.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\..\config.h"

class EKF : public SLAMModule {
public:
private:
    LandmarkExtractor* landmarkExtractor = nullptr;
    
public:
    EKF();
    ~EKF();

    void InitSlam(double startX, double startY, double startTheta);
    void UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud);
private:

};

#endif