#ifndef CONFIG_H_
#define CONFIG_H_

#include <utility>

// Possible SLAM options
#define SLAM_OPTION_EKF 0
#define SLAM_OPTION_GMAPPING 1

const int STARTING_MAP = 1;
const int STARTING_SLAM = SLAM_OPTION_GMAPPING;
const bool SHOW_POSSIBLE_STARTING_LOCATIONS = false;

// Sensor Model Details; 360 degrees
const int SENSOR_MODEL_POINTS_PER_SCAN = 1455; // 1455 default (8000 / 5.5Hz)
const double SENSOR_MODEL_TIME_PER_SCAN = 1.0 / 5.5; // default 5.5 Hz
const int SENSOR_MODEL_MEASURE_MIN = 150;   // 150 default
const int SENSOR_MODEL_MEASURE_MAX = 12000; // 12000 default

// Period details (except sensor ^^^)
const double GRAPHICS_FPS = 1.0 / 60.0; // Also how often motion is updated
const double MOTION_PERIOD = GRAPHICS_FPS; // No point having it faster than graphics
const int SLAM_MINIMUM_PERIOD_COUNT = 11; // At least how many frames must pass before the algorithm considers running (in tangent with slam_period) consider keeping at a minimum of scan period (10.9)
const int SLAM_MAXIMUM_PERIOD_COUNT = 60; // The maximum frames that can pass before the algorithm must run
const double SLAM_MINIMUM_DISTANCE = 50; // must move 50 mm before running full algorithm

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
const double MOTION_MODEL_ACCELERATION = 500.0 * MOTION_PERIOD; // mm/s^2
const double MOTION_MODEL_MAX_VELOCITY = 300.0 * MOTION_PERIOD; // mm/s
const double MOTION_MODEL_ROTATION = 1.0 * MOTION_PERIOD; // 1 rad/s, or 57.2 degrees per second
const double MOTION_MODEL_FORWARD_DEVIATION = 0.05; //5%: hardwood floor
const double MOTION_MODEL_FORWARD_ROTATION_DEVIATION = 0.02 / 3; // +- 0.02 within 3 sigma (fixed)
const double MOTION_MODEL_ROTATION_DEVIATION = 0.02; //2%: hardwood floor
const double MOTION_MODEL_ROTATION_FIXED = MOTION_MODEL_ROTATION_DEVIATION * MOTION_MODEL_ROTATION;

// Gmapping Configurations
const int GMAPPING_NUM_PARTICLES = 30;
const double GMAPPING_GRID_CELL_SIZE = 50; // 50 mm
const int GMAPPING_SECTOR_SIZE = 16; //each sector is 16 x 16 cells
const int GMAPPING_SECTOR_NUM_CELLS = GMAPPING_SECTOR_SIZE*GMAPPING_SECTOR_SIZE;
const int GMAPPING_HISTORY_SIZE = SLAM_MAXIMUM_PERIOD_COUNT * 2;
const double GMAPPING_MAX_LOG_ODDS = 10.0; // +- this much
const double GMAPPING_LOG_ODDS_HIT = 0.85;//tmp
const double GMAPPING_LOG_ODDS_MISS = -0.4;//tmp

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
const float GMAPPING_CELL_SIZE_DIPS = (float)(GMAPPING_GRID_CELL_SIZE / MM_PER_DIP);

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

const int COLOR_PALETTE_WALLS_SIZE = 10;
const unsigned int COLOR_PALETTE_WALLS[COLOR_PALETTE_WALLS_SIZE] = {
    0xFFE1E1E1, //1: 225
    0xFFC8C8C8, //2: 200
    0xFFAFAFAF, //3: 175
    0xFF969696, //4: 150
    0xFF7D7D7D, //5: 125
    0xFF646464, //6: 100
    0xFF4B4B4B, //7: 75
    0xFF323232, //8: 50
    0xFF191919, //9: 25
    0xFF000000, //10: 0
};

const int COLOR_WALL_BRUSH = 0xFFFFFFFF; // fear made me make this. Fear and ignorance. Powerful combo.

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

// Sensor
const int SENSOR_NUM_TRACKED_PACKETS = 4;

// AI/Motion
enum class RobotCommand { //do we add none?
    FORWARD,
    STOP,
    LEFT,
    RIGHT
};

#endif