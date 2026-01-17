all: main

main: main.cpp
	g++ -Wall -Wconversion -o main.exe main.cpp AI\ai.cpp Graphics\graphics.cpp Listener\listener.cpp Models\motionModel.cpp Models\robotModel.cpp Models\sensorModel.cpp Simulator\simulator.cpp SLAMModels\EKF\ekf.cpp SLAMModels\EKF\landmarkExtractor.cpp SLAMModels\Gmapping\gmapping.cpp SLAMModels\Gmapping\particle.cpp SLAMModels\Gmapping\scanMatcher.cpp SLAMModels\Templates\slam.cpp World\world.cpp World\map.cpp World\Objects\oline.cpp World\Objects\opoint.cpp  -ld2d1 -ldxguid -ldxgi -ld3d11 -lgdi32 -luser32 -ldwrite