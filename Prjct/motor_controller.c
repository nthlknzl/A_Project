#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "msgbus/messagebus.h"

#include <motors.h>
#include <position_awareness.h>
#include <motor_controller.h>
#include <navigation.h>

#define BASE_SPEED 500 // was initially 500 steps/s
#define TURN_TIME 800 // in ms

/* General concept
 * ---------------
 *
 *  The motor control thread is called periodically and calls oxiliary functions
 *  to update the speed variables (speed_left and speed_right). Then, the motors
 *  are updated with the speed.
 *
 * The current state is received via the messagebus.
 *
 * */


// store the speed of the motors. This variables are used to transfer information between some functions in this file!
static int16_t speed_left = 0;
static int16_t speed_right = 0;

extern messagebus_t bus;

// constants for the controller
#define Kp 0.1
#define  Ki 0.0001
#define Kd 0.2



static THD_WORKING_AREA(waMotorController, 256);
static THD_FUNCTION(MotorController, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    // read the current state in the bus.
    motion_state state; // variable to store the state - read from the message bus
    messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/motor_state");

    while(1){
        time = chVTGetSystemTime();
        messagebus_topic_read(state_topic, &state, sizeof(state)); // we use topic-read and not topicWait in order to allow the motor_controller to run more frequent than the system control thread. This is useful for the PID controller in this thread.

        // update the speed variables given the state
        switch(state){
        	case FORWARD_MOTION: control_forward_motion(); break;
        	case STOP: stop_motors(); break;
        	case LEFT_TURN: turn(LEFT_TURN); break;
        	case RIGHT_TURN: turn(RIGHT_TURN); break;
        	default: control_forward_motion(); // in case there's an undefined state somewhere.
            }

        update_motors(); // write the speed variables to the motor

        //40Hz
        chThdSleepUntilWindowed(time, time + MS2ST(25));
    }
}
/*
 * implements a PID controller with the error between the left and right proximity sensor
 * sets the speed_left and speed_right variables as output
 */
void control_forward_motion( void ){
	// get left-right error
	int16_t lr_error = get_left_right_error();

	int16_t control_value = 0; // in the end: speed_left = BASE_SPEED + control_value and speed_right = BASE_SPEED - control_value

    // P
	control_value += Kp * lr_error;

	//D
	static int16_t previous_lr_error = 0;
	control_value += Kd * (lr_error-previous_lr_error);

	// I
	static int16_t integral_error = 0;
	integral_error += lr_error;
	/*if(integral_error > BASE_SPEED/Ki){integral_error = 0.2*BASE_SPEED/Ki;}
	else if(integral_error < - BASE_SPEED/Ki){integral_error = -0.2*BASE_SPEED/Ki;} */ // deleted bc of overflow and prob. not needed
	control_value += Ki * integral_error;

	// set previous error for next call
	previous_lr_error = lr_error;

	// set the speed variables
	speed_left = BASE_SPEED + control_value;
	speed_right = BASE_SPEED - control_value;

}

/*
 * sets speed_left and speed_right to stop the motor
 */
void stop_motors( void ){
	speed_left = 0;
	speed_right = 0;
}

/* sets the speed_left and speed_right for a turn
 * @input turn_direction direction : enum with the values LEFT_TURN or RIGHT_TURN determining which way to turn
 * in case an other enum value is received the function does nothing.
 */
void turn(motion_state direction){
	switch(direction){
		case LEFT_TURN: speed_left = BASE_SPEED; speed_right = -BASE_SPEED; break;
		case RIGHT_TURN: speed_left = -BASE_SPEED; speed_right = BASE_SPEED; break;
		default: break;
	}
}

/*
 * uses the variables speed_left and speed_right to set the motor speed while controling the speed limit.
 */
void update_motors( void ){
    // check that the speed is within the allowed limits
    if(speed_left > MOTOR_SPEED_LIMIT){speed_left = MOTOR_SPEED_LIMIT;}
    else if(speed_left < -MOTOR_SPEED_LIMIT){speed_left = -MOTOR_SPEED_LIMIT;}
    if(speed_right > MOTOR_SPEED_LIMIT){speed_right = MOTOR_SPEED_LIMIT;}
    else if(speed_right < -MOTOR_SPEED_LIMIT){speed_right = -MOTOR_SPEED_LIMIT;}


    //applies the speed to the motors
    right_motor_set_speed(speed_right);
    left_motor_set_speed(speed_left);
}


/*
 * Start the thread
 */
void motor_controller_start(void){
	chThdCreateStatic(waMotorController, sizeof(waMotorController), NORMALPRIO+20, MotorController, NULL);
}

