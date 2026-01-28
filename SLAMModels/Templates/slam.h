#ifndef SLAM_MODELS_TEMPLATES_SLAM_H_
#define SLAM_MODELS_TEMPLATES_SLAM_H_

#include <vector>
#include <mutex>
#include <memory>

#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\..\config.h"

class SLAMModule {
protected:
    std::vector<float>* renderMap = nullptr;
    std::shared_ptr<std::mutex> guardRenderMap;

public:
    explicit SLAMModule();
    virtual ~SLAMModule();

    virtual void InitSlam(double startX, double startY, double startTheta) = 0;
    virtual void UpdateSlam(RobotCommand command, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) = 0;
    virtual void GetPose(double& x, double& y, double& theta) = 0;

    std::vector<float>** GetRenderMapAddress();
    std::shared_ptr<std::mutex> GetRenderMapGuard();
private:
};

#endif