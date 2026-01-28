#ifndef SLAM_MODELS_GMAPPING_H_
#define SLAM_MODELS_GMAPPING_H_

#include <windows.h>
#include <utility>
#include <atomic>
#include <unordered_set>

#include "..\Templates\slam.h"
#include "particle.h"
#include "scanMatcher.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "statePacket.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\..\config.h"

class Gmapping : public SLAMModule {
public:
private:
    ScanMatcher* scanMatcher = nullptr;

    Particle** particles = nullptr;

    // Needed for P(Xt | Xt-1, u)?
    std::pair<RobotCommand, double>* commandHistory = nullptr;
    int nextCommand = 0;

    HANDLE slamSemaphore;
    std::thread slamThread;
    std::atomic<bool> slamFinished;

    int ticksSinceLastUpdate = 0;

    StatePacket* currPacket = nullptr;

    // How far would the robot have gone if there was no noise?
    //   Used for determining if slam should run given min distance
    double accumulatedPoseSinceLastUpdate = 0.0;

    double lastScanExpX = 0.0;
    double lastScanExpY = 0.0;
    double lastScanExpTheta = 0.0;

    double accumulatingExpX = 0.0;
    double accumulatingExpY = 0.0;
    double accumulatingExpTheta = 0.0;

    double lastScanTimestamp = 0.0;

    //for render purposes. No other function will access this!
    double startX = 0.0;
    double startY = 0.0;
    double startTheta = 0.0;

public:
    Gmapping();
    ~Gmapping();

    void InitSlam(double startX, double startY, double startTheta);

    void UpdateSlam(RobotCommand command, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud);
    void GetPose(double& x, double& y, double& theta);
private:
    void AddToHistory(RobotCommand command, double timstamp);

    void RefineEstimates();

    void UpdateMaps();
    void AddAffectedCells(int startX, int startY, int endX, int endY, std::unordered_set<std::pair<int, int>, pair_hash>& misses, std::unordered_set<std::pair<int, int>, pair_hash>& hits);

    void CreateRenderCopy(int particleIndex);
    std::pair<float, float> XYToDips(double x, double y);
};

#endif