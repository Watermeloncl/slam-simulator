#ifndef CONFIG_H_
#define CONFIG_H_

// Possible SLAM options
#define SLAM_OPTION_EKF 0
#define SLAM_OPTION_GMAPPING 1

const int STARTING_MAP = 3;
const int STARTING_SLAM = SLAM_OPTION_EKF;

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

const float MM_PER_DIP = 18.75;

// Colors
// To add brush/color, you must:
//  1) increase size
//  2) add color to const ints
//  3) add that const int to value set
const int COLOR_PALETTE_SIZE = 3; //4

const int COLOR_PALETTE_BACKGROUND = 0xe3e3e3;
// const int COLOR_PALETTE_RED = 0xFF0000;
const int COLOR_PALETTE_WHITE = 0xFFFFFF;
const int COLOR_PALETTE_BLACK = 0x000000;

const int COLOR_PALETTE_VALUES[COLOR_PALETTE_SIZE] = {
    COLOR_PALETTE_BACKGROUND,
    // COLOR_PALETTE_RED,
    COLOR_PALETTE_WHITE,
    COLOR_PALETTE_BLACK
};

// Lines, Shapes, Sizes
const int BACKGROUND_LINE_WIDTH = 4;
const int MAP_LINE_WIDTH = 2;

// Map Specs
const int MAX_MAP_LINES = 64;

// Quadrants
const int TOP_LEFT = 1;
const int TOP_RIGHT = 2;
const int BOTTOM_LEFT = 3;
const int BOTTOM_RIGHT = 4;

#endif