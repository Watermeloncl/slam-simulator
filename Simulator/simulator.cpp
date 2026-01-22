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
    double accumulator = 0.0;
    //TODO change back to 60 fps, just only scan every x movement
    // thread scanning
    const double dt = SENSOR_MODEL_TIME_PER_SCAN;
    //const double dt = 1.0 / 60.0;



    MSG msg = {};

    this->robotModel->InitializeRobot(this->world->GetMap());

    RenderPacket* renderPacket = new RenderPacket(
        this->robotModel->GetRealX(),
        this->robotModel->GetRealY(),
        this->robotModel->GetRealTheta(),
        nullptr
    );
            
    this->graphicsModule->UpdateRenderInfo(renderPacket);

    int frame = 0;
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
        accumulator += elapsed.count();

        while(accumulator >= dt) {
            frame++;

            //command = GetNextCommand();
            //refinedCommand = CommandRobot();

            this->robotModel->DummyUpdate(); //tmp

            auto start = std::chrono::high_resolution_clock::now();
            PointCloud* pointCloud = this->robotModel->GetScan();
            OPoint** renderPointCloud = this->robotModel->GetRenderScan();
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;

            //UpdateSlam(pointCloud);
            delete pointCloud;

            //map = GetMap();
            //pose = GetPose();
            //reality = GetTruePose();
            //Trajectory = UpdateTrajectory();

            RenderPacket* renderPacket = new RenderPacket(
                this->robotModel->GetRealX(),
                this->robotModel->GetRealY(),
                this->robotModel->GetRealTheta(),
                renderPointCloud
            );
            
            this->graphicsModule->UpdateRenderInfo(renderPacket);
            
            accumulator -= dt;
        }

        this->graphicsModule->RenderFrame();
    }
}