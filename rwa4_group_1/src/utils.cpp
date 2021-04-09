#include "utils.h"

std::unordered_map<std::string, double> model_height = {
        {"piston_rod_part_red", 0.0065}, // modified because it sinks into the surface a bit
        {"piston_rod_part_green", 0.0065},
        {"piston_rod_part_blue", 0.0065},
        {"pulley_part_red", 0.07},
        {"pulley_part_green", 0.07},
        {"pulley_part_blue", 0.07},
        {"gear_part_red", 0.012},
        {"gear_part_green", 0.012},
        {"gear_part_blue", 0.012},
        {"gasket_part_red", 0.02},
        {"gasket_part_green", 0.02},
        {"gasket_part_blue", 0.02},
        {"disk_part_red", 0.029},
        {"disk_part_green", 0.029},
        {"disk_part_blue", 0.029}
};


std::unordered_map<int, std::string> sensorLocationMap = {
        {0, AGV1}, 
        {1, AGV2},
        {2, ""}, // CONV_BELT
        {3, BINS},
        {4, BINS},
        {5, BINS},
        {6, BINS},
        {7, SHELF_1},
        {8, SHELF_1},
        {9, SHELF_2},
        {10, SHELF_2},
        {11, SHELF_8},
        {12, SHELF_8},
        {13, SHELF_5},
        {14, SHELF_5},
        {15, SHELF_11},
        {16, SHELF_11}
};


std::unordered_map<std::string, std::string> oppositeAGV = {{AGV1_ID, AGV2_ID}, {AGV2_ID, AGV1_ID}, {ANY_AGV, AGV2_ID}}; // logic for "any" should be changed
std::unordered_map<std::string, std::string> agvTrayMap = {{AGV1_ID, AGV1_TRAY}, {AGV2_ID, AGV2_TRAY}}; 
