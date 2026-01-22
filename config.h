#ifndef CONFIG_H_
#define CONFIG_H_

#include <utility>

// Possible SLAM options
#define SLAM_OPTION_EKF 0
#define SLAM_OPTION_GMAPPING 1

const int STARTING_MAP = 2;
const int STARTING_SLAM = SLAM_OPTION_EKF;
const bool SHOW_POSSIBLE_STARTING_LOCATIONS = false;

// Sensor Model Details; 360 degrees
const int SENSOR_MODEL_POINTS_PER_SCAN = 1455; // 1455 default (8000 / 5.5Hz)
const double SENSOR_MODEL_TIME_PER_SCAN = 1.0 / 5.5; // default (1.0 second / 5.5 Hz)
const int SENSOR_MODEL_MEASURE_MIN = 150;   // 150 default
const int SENSOR_MODEL_MEASURE_MAX = 12000; // 12000 default

// Defaults based on lidar specs {1% <= 3000, 2% 3-5, 2.5% 5+}. Be sure to cover measurement range.
const int SENSOR_MODEL_ACCURACY_TIERS = 3;
const std::pair<double, double> SENSOR_MODEL_ACCURACY[SENSOR_MODEL_ACCURACY_TIERS] = {
    {0.01, 3000},
    {0.02, 5000},
    {0.025, 25000}
};

// Default based on lidar specs is 1% < 12 meters. For simplicity, one range resolution for all distances
const double SENSOR_MODEL_RANGE_RESOLUTION = 0.01;

// Motion Model Details
const double MOTION_MODEL_ACCELERATION = 500.0; // mm/s^2
const double MOTION_MODEL_MAX_VELOCITY = 300.0; // mm/s


///////////////////////////////////////////////
//// Do not touch any parameters here down ////
///////////////////////////////////////////////

// Graphics
// Screen Size
const int CLIENT_SCREEN_WIDTH = 1280;
const int CLIENT_SCREEN_HEIGHT = 720;

// World in mm
const int WORLD_SIZE_METERS_WIDTH = 12000;
const int WORLD_SIZE_METERS_HEIGHT = 6750;

const double MM_PER_DIP = 18.75;

// Colors
// To add brush/color, you must:
//  1) increase size
//  2) add color to const ints
//  3) add that const int to value set
const int COLOR_PALETTE_SIZE = 4;

const int COLOR_PALETTE_BACKGROUND = 0xe3e3e3;
const int COLOR_PALETTE_RED = 0xFF0000;
const int COLOR_PALETTE_WHITE = 0xFFFFFF;
const int COLOR_PALETTE_BLACK = 0x000000;

const int COLOR_PALETTE_VALUES[COLOR_PALETTE_SIZE] = {
    COLOR_PALETTE_BACKGROUND,
    COLOR_PALETTE_RED,
    COLOR_PALETTE_WHITE,
    COLOR_PALETTE_BLACK
};

// Lines, Shapes, Sizes
const int BACKGROUND_LINE_WIDTH = 4;
const int MAP_LINE_WIDTH = 2;

const int ROBOT_RADIUS = 8;

const double LIDAR_POINT_RADIUS = 2.0;

// Map Specs
const int MAX_MAP_LINES = 64;
const int MAX_MAP_STARTS = 16;

// Quadrants
const int TOP_LEFT = 1;
const int TOP_RIGHT = 2;
const int BOTTOM_LEFT = 3;
const int BOTTOM_RIGHT = 4;

#endif