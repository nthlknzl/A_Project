#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <navigation.h>

typedef enum {LEFT, RIGHT} turn_direction; // bruchts die variable??

void motor_controller_start(void);
void motor_controller_test( void );

// control functions
void update_motors( void );
void update_speed( enum motion_state state);

// states
void control_forward_motion( void );
void stop_motors( void );
void turn(turn_direction direction);

#endif
