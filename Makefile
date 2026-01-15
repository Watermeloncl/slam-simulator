all: main

main: main.cpp
	g++ -Wall -Wconversion -o main.exe main.cpp Graphics\graphics.cpp Listener\listener.cpp Simulator\simulator.cpp -ld2d1 -lgdi32 -luser32 -ldwrite