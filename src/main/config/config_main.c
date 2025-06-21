#include "config_main.h"

const float STATUS_PATTERNS_HSV[NUM_STATUS_PATTERNS][4] = {
    {0.0, 0.0, 0.0, 0.5},       // IDLE: off
    {120.0, 255.0, 80, 1.0}, // TRACKING: Green
    {240.0, 255.0, 80, 1.0}, // HOMING: Blue
    {240.0, 255.0, 80, 0.2}, // NIGHT: Blue
    {0.0, 255.0, 80, 0.2},   // STOP: RED
    {60.0, 255.0, 80, 1.0},  // WARNING: Yellow
    {0.0, 255.0, 80, 1.0},   // ERROR: RED
    {60.0, 255.0, 80, 0.2},  // UNKNOWN: Yellow
};