#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <motor_controller.h> // for the state enum

typedef uint8_t surrounding; // stores the information about walls in proximity. The four used bits are described below

// 1 at bit X: there is a wall in fron (bit 1), left (bit 2), right (bit 3), or there was a line detected (bit 8)
// multiple bits can be set at the same time. Bits are not cleared by reading functions.
#define WALL_IN_FRONT_BIT 0b00000001
#define WALL_LEFT_BIT 0b00000010
#define WALL_RIGHT_BIT 0b00000100
#define FLOOR_LINE_IN_FRONT 0b10000000

// State of motion: This is used to set the motor_controller mode
enum motion_state {FORWARD_MOTION, STOP, LEFT_TURN, RIGHT_TURN};
enum detection_state {NOTHING_DETECTED, EDGE_DETECTED, LINE_DETECTED, RED_DETECTED};

void navigation_thread_start(void);
void command_turn(enum motion_state direction); // command a left or right turn
void command_motor( enum motion_state command ); // send a command to the motor

#endif
