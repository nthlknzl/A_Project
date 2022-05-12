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
    enum motion_state state; // variable to store the state - read from the message bus
    messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/motor_state");

    while(1){
        time = chVTGetSystemTime();
        messagebus_topic_read(state_topic, &state, sizeof(state)); // we use topic-read and not topicWait in order to allow the motor_controller to run more frequent than the system control thread. This is useful for the PID controller in this thread.

        update_speed(state);
        update_motors();

        //chprintf((BaseSequentialStream *)&SD3, "left: %d \t right: %d \r\n", speed_left, speed_right);

        //20Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}
/*
 * implements a PID controller with the error between the left and right proximity sensor
 */
void control_forward_motion( void ){
	// get left-right error
	int16_t lr_error = get_left_right_error();

	speed_left = BASE_SPEED;
	speed_right = BASE_SPEED;

    // P
	speed_left += Kp * lr_error;
	speed_right -= Kp * lr_error;

	//D
	static int16_t previous_lr_error = 0;
	speed_left += Kd * (lr_error-previous_lr_error);
	speed_right -= Kd * (lr_error-previous_lr_error);

	// I
	static int16_t integral_error = 0;
	integral_error += lr_error;
	/*if(integral_error > BASE_SPEED/Ki){integral_error = 0.2*BASE_SPEED/Ki;}
	else if(integral_error < - BASE_SPEED/Ki){integral_error = -0.2*BASE_SPEED/Ki;} */ // deleted bc of overflow and prob. not needed
	speed_left += Ki * integral_error;
	speed_right -= Ki * integral_error;

	// set previous error for next call
	previous_lr_error = lr_error;
}

/*
 * sets the variables to stop the motor
 */
void stop_motors( void ){
	speed_left = 0;
	speed_right = 0;
}

/* sets the speed_left and speed_right for a turn
 * @input turn_direction tirection : enum with the values LEFT or RIGHT determining which way to turn
 */
void turn(turn_direction direction){
	switch(direction){
		case LEFT: speed_left = BASE_SPEED; speed_right = -BASE_SPEED; break;
		case RIGHT: speed_left = -BASE_SPEED; speed_right = BASE_SPEED; break;
	}
}

/*
 * uses the variables speed_left and speed_right to set the motor speed.
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
 * @ input enum motion_state state : the current state (forward, left_turn, right_turn, or stop) of the system
 * The function updates updates the variables speed_left and speed_right according to the current state.
 */
void update_speed( enum motion_state state){
    // check state and execute it - this sets the motor speed variables
    switch(state){
    	case FORWARD_MOTION: control_forward_motion(); break;
    	case STOP: stop_motors(); break;
    	case LEFT_TURN: turn(LEFT); break;
    	case RIGHT_TURN: turn(RIGHT); break;
    	default: control_forward_motion(); // in case there's an undefined state somewhere.
        }
}



/*
 * Start the thread
 */
void motor_controller_start(void){
	chThdCreateStatic(waMotorController, sizeof(waMotorController), NORMALPRIO+2, MotorController, NULL);
}

/*
 * Test function for the motor. comment or uncomment lines to perform tests.
 */
void motor_controller_test( void ){
	/*
	 * This test sets the motor_control threat to high priority (+20) to ensure its isolation from the other
	 * threads before cycling through the states:
	 * 1. forward
	 * 2. stop
	 * 3. left turn
	 * 4. right turn
	 *
	 */

	// init the messagebus
	enum motion_state initital_state = FORWARD_MOTION;
	messagebus_topic_t motor_topic;
	MUTEX_DECL(motor_topic_lock);
	CONDVAR_DECL(motor_topic_condvar);
	messagebus_topic_init(&motor_topic, &motor_topic_lock, &motor_topic_condvar, &initital_state, sizeof(initital_state));
	messagebus_advertise_topic(&bus, &motor_topic, "/motor_state");

	// create a messagebus topic to publish information about the surrounding
	surrounding_walls_info surrounding = NO_WALLS;
	messagebus_topic_t surrounding_topic;
	MUTEX_DECL(surrounding_topic_lock);
	CONDVAR_DECL(surrounding_topic_condvar);
	messagebus_topic_init(&surrounding_topic, &surrounding_topic_lock, &surrounding_topic_condvar, &surrounding, sizeof(surrounding));
	messagebus_advertise_topic(&bus, &surrounding_topic, "/surrounding");


	// start the thread with high priority
	chThdCreateStatic(waMotorController, sizeof(waMotorController), NORMALPRIO + 20, MotorController, NULL);
	navigation_thread_start();

	/*
	 * Test 1: cycle through all the states
	 */
	/*
	while(1){
		//enum motion_state {FORWARD_MOTION, STOP, LEFT_TURN, RIGHT_TURN}; // temporary definition
		for(enum motion_state state = FORWARD_MOTION; state<=RIGHT_TURN; state++){
			messagebus_topic_publish(&motor_topic, &state, sizeof(state));
			chThdSleepMicroseconds(2000000);
		}
	}
*/

	/*
	 * Test 2: Forward only
	 */

	/*
	while(1){
		enum motion_state state = FORWARD_MOTION;
		messagebus_topic_publish(&motor_topic, &state, sizeof(state));
		chThdSleepMicroseconds(700000);
	}
	*/

	/*
	 * Test 3: left square
	 */

	/*
	enum motion_state state = FORWARD_MOTION;
	while(1){
		state = FORWARD_MOTION;
		messagebus_topic_publish(&motor_topic, &state, sizeof(state));
		chThdSleepMicroseconds(4000000);

		state = LEFT_TURN;
		messagebus_topic_publish(&motor_topic, &state, sizeof(state));
		chThdSleepMicroseconds(700000);

	}
	*/

	/*
	 * Test 4: use the navigation thread
	 */


	//navigation_thread_start();

	enum motion_state state = FORWARD_MOTION;
	messagebus_topic_publish(&motor_topic, &state, sizeof(state));
	chThdSleepMicroseconds(1000000);

	while(1){
		chThdSleepMicroseconds(1000000);
	}

}

