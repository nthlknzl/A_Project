#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "msgbus/messagebus.h"

#include <motors.h>
#include <position_awareness.h>
#include <motor_controller.h>

#define BASE_SPEED 500
#define FORWARD_TIME_AFTER_TURN 3000 // in ms
#define TURN_TIME 1000 // in ms

/* General consept
 * ---------------
 *
 * The motor control acts differently depending on the current state.
 *
 *
 *
 *
 * */

enum motion_state {FORWARD_MOTION, STOP, LEFT_TURN, RIGHT_TURN};

static int16_t speed_left = 0;
static int16_t speed_right = 0;

extern messagebus_t bus;

// constants for the controller
#define Kp 0.1
#define  Ki 0.1



static THD_WORKING_AREA(waMotorController, 256);
static THD_FUNCTION(MotorController, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    // read the current state in the bus.
    enum motion_state state; // variable to store the state - read from the message bus
    messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/state");
    messagebus_topic_read(state_topic, &state, sizeof(state)); // we use topic-read and not topicWait in order to allow the motor_controller to run more frequent than the system control thread. This is useful for the PID controller in this thread.

    speed_left = 0;
    speed_right = 0;

    while(1){
        time = chVTGetSystemTime();

        // check state and execute it - this sets the motor speed variables
        switch(state){
        	case FORWARD_MOTION: control_forward_motion(); break;
        	case STOP: control_forward_motion(); break;
        	case LEFT_TURN: control_forward_motion(); break;
        	case RIGHT_TURN: control_forward_motion(); break;
            }

        // check that the speed is within the allowed limits
        if(speed_left > MOTOR_SPEED_LIMIT){speed_left = MOTOR_SPEED_LIMIT;}
        else if(speed_left < -MOTOR_SPEED_LIMIT){speed_left = -MOTOR_SPEED_LIMIT;}
        if(speed_right > MOTOR_SPEED_LIMIT){speed_right = MOTOR_SPEED_LIMIT;}
        else if(speed_right < -MOTOR_SPEED_LIMIT){speed_right = -MOTOR_SPEED_LIMIT;}


        //applies the speed to the motors
        right_motor_set_speed(speed_right);
        left_motor_set_speed(speed_left);

        chprintf((BaseSequentialStream *)&SD3, "left: %d \t right: %d \r\n", speed_left, speed_right);

        //20Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}


void control_forward_motion( void ){
	// get left-right error
	int16_t lr_error = get_left_right_error();

	speed_left = BASE_SPEED;
	speed_right = BASE_SPEED;

    // P
	speed_left += Kp * lr_error;
	speed_right -= Kp * lr_error;
	// I
	static int16_t integral_error = 0;
	integral_error += lr_error;
	if(integral_error > BASE_SPEED/Ki){integral_error = 0.2*BASE_SPEED/Ki;}
	else if(integral_error < - BASE_SPEED/Ki){integral_error = -0.2*BASE_SPEED/Ki;}
	speed_left += Ki * integral_error;
	speed_right -= Ki * integral_error;
}

void stop_motors( void ){
	speed_left = 0;
	speed_right = 0;
}
void turn(turn_direction direction){
	switch(direction){
		case LEFT: speed_left = BASE_SPEED; speed_right = -BASE_SPEED; break;
		case RIGHT: speed_left = -BASE_SPEED; speed_right = BASE_SPEED; break;
	}
}



/*
 * Start the thread
 */
void motor_controller_start(void){
	chThdCreateStatic(waMotorController, sizeof(waMotorController), NORMALPRIO, MotorController, NULL);
}

