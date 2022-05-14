#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <navigation.h>

void motor_controller_start(void);
void motor_controller_test( void );

// control functions
void update_motors( void );
void update_speed(motion_state state);

// states
void control_forward_motion( void );
void stop_motors( void );
void turn(motion_state direction);

#endif
