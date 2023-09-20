#ifndef MAIN_H
#define MAIN_H

#include <sstream>

#define ENC_CPR 64
#define GEAR_RATIO 18.75
#define WHEEL_RADIUS 0.035
#define WHEEL_POS_R 0.145
#define PID_KP 0.8
#define PID_TI 0.05
#define PID_TD 0.0
#define PID_DELTA_T 0.01
#define MAIN_DELTA_T 0.02 // ! Todo cpu load

#define MAX_CMD_ARGS 5
#define MOTOR_COUNT 3
#define CMD_TIMEOUT_MS 1000

#define PID_TYPE_SPEED

#define ENABLE_PID_Z
// #define ENABLE_PID_X
// #define ENABLE_PID_Y


void pc_rx_callback();

void check_for_timeout();

void processPacket(const std::string &packet);

#endif