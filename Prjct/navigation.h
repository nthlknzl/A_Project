#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <motor_controller.h> // for the state enum

typedef uint8_t surrounding;

#define WALL_IN_FRONT_BIT 0b00000001
#define WALL_LEFT_BIT 0b00000010
#define WALL_RIGHT_BIT 0b00000100
#define LINE_IN_FRONT 0b10000000

void navigation_thread_start(void);
void command_turn(enum motion_state direction);
void command_motor( enum motion_state command );

#endif
