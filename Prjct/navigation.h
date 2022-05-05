#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <motor_controller.h> // for the state enum
void navigation_thread_start(void);
void command_turn(enum motion_state direction);



#endif
