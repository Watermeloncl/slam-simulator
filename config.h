#ifndef CONFIG_H_
#define CONFIG_H_

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




#endif