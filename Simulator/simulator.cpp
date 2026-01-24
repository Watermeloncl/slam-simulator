#include <chrono>
#include <iostream>

#include "simulator.h"
#include "..\Graphics\graphics.h"
#include "..\Graphics\renderPacket.h"
#include "..\World\world.h"
#include "..\World\Objects\opoint.h"
#include "..\Models\robotModel.h"
#include "..\Models\Lidar\pointCloud.h"
#include "..\AI\ai.h"
#include "..\SLAMModels\Templates\slam.h"
#include "..\SLAMModels\EKF\ekf.h"
#include "..\SLAMModels\Gmapping\gmapping.h"
#include "..\config.h"

using SchedClock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;
using TimeStamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

Simulator::Simulator(HINSTANCE hInstance, int nCmdShow) {
    this->graphicsModule = new GraphicsModule(hInstance, nCmdShow);
    
    this->world = new World();
    this->graphicsModule->CreateBackground(this->world->GetMap());

    this->robotModel = new RobotModel();
    this->aiModule = new AIModule();

    if(STARTING_SLAM == SLAM_OPTION_EKF) {
        this->slamModule = new EKF();
    } else {
        this->slamModule = new Gmapping();
    }
}

Simulator::~Simulator() {
    delete this->graphicsModule;
    delete this->world;
    delete this->robotModel;
}

void Simulator::RunMainLoop() {
    TimeStamp lastTime = SchedClock::now();
    TimeStamp now = SchedClock::now();

    Duration elapsed;
    double scanAccumulator = 0.0;
    const double scanPeriod = SENSOR_MODEL_TIME_PER_SCAN; //tmp

    double slamAccumulator = 0.0;
    const double slamPeriod = SLAM_PERIOD;

    double mainAccumulator = 0.0;
    const double mainPeriod = GRAPHICS_FPS;

    double motionAccumulator = 0.0;
    double motionPeriod = MOTION_PERIOD;

    double totalAccumulator = 0.0;

    MSG msg = {};

    this->robotModel->InitializeRobot(this->world->GetMap());

    RenderPacket* renderPacket = new RenderPacket(
        this->robotModel->GetRealX(),
        this->robotModel->GetRealY(),
        this->robotModel->GetRealTheta(),
        nullptr
    );
            
    this->graphicsModule->UpdateRenderInfo(renderPacket);

    bool running = true;

    while(running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) running = false;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        //act on messages
        // - pause (space)
        // - reset (r)
        // - change map and reset (m)
        // - probability toggle (p)
        // - algorithm change (a)
        // - Change Trajectory (click top right)

        now = SchedClock::now();
        elapsed = now - lastTime;
        lastTime = now;
        motionAccumulator += elapsed.count();
        scanAccumulator += elapsed.count();
        slamAccumulator += elapsed.count();
        mainAccumulator += elapsed.count();
        totalAccumulator += elapsed.count();


        while(motionAccumulator >= motionPeriod) {
            //update AI
            
            RobotCommand initialCommand = this->aiModule->GetCommand();
            /*RobotCommand finalCommand = */this->robotModel->CommandRobot(initialCommand);

            // std::cout << "pos: " << this->robotModel->GetRealX() << " " << this->robotModel->GetRealY() << " " << this->robotModel->GetRealTheta() << std::endl;


            motionAccumulator -= motionPeriod;
        }

        while(scanAccumulator >= scanPeriod) {
            this->robotModel->KickOffScan(totalAccumulator); //what if it can't?
            scanAccumulator -= scanPeriod;
        }

        while(slamAccumulator >= slamPeriod) {
            //run slam algorithm update
            //uses refined command?
            PointCloud* pointCloud = this->robotModel->CopyLatestScan();

            //updateSLAM(refinedCommand, pointCloud) // must check if pointCloud isn't nullptr

            delete pointCloud; //delete copy made
            slamAccumulator -= slamPeriod;
        }

        while(mainAccumulator >= mainPeriod) {
            //get updated graphics
            //map = GetMap();
            //pose = GetPose();
            //Trajectory = UpdateTrajectory();

            // save cpu cycles by not making copy
            OPoint** renderPointCloud = this->robotModel->CopyLatestRenderScan();
            // std::cout << "checkpoint 1" << std::endl;

            RenderPacket* renderPacket = new RenderPacket(
                this->robotModel->GetRealX(),
                this->robotModel->GetRealY(),
                this->robotModel->GetRealTheta(),
                renderPointCloud
            );
            
            this->graphicsModule->UpdateRenderInfo(renderPacket);

            mainAccumulator -= mainPeriod;
        }

        this->graphicsModule->RenderFrame();
    }
}