#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

typedef enum {LEFT, RIGHT} turn_direction;

enum motion_state {FORWARD_MOTION, STOP, LEFT_TURN, RIGHT_TURN}; // temporary definition


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
