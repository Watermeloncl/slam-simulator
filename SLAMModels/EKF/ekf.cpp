#include "ekf.h"
#include "..\Templates\slam.h"
#include "landmarkExtractor.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\..\config.h"

EKF::EKF() : SLAMModule() {
    this->landmarkExtractor = new LandmarkExtractor();
}

EKF::~EKF() {

}

void EKF::InitSlam(double startX, double startY, double startTheta) {
    
}

void EKF::UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) {
    // Check if pointcloud is nullptr
    // Estimate and update pose
    // Estimate delta xytheta and send to landmark
    //    extractor, with point cloud
    // - Convert polar coordinates to cartesian and account
    //    for motion blur
    // - Cluster according to distance and cluster threshold
    // - For each cluster, use line split and merge
    // - For each cluster, determine which lines should be one
    // - Determine intersection points
    // - Pass intersection points back to EKF
    // 

    //MUST DELETE POINT CLOUD
    delete pointCloud;
}
