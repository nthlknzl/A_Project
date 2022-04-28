typedef enum {LEFT, RIGHT} turn_direction;

void motor_controller_start(void);

// states
void control_forward_motion( void );
void stop_motors( void );
void turn(turn_direction direction);
