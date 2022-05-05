#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "msgbus/messagebus.h"

#include <navigation.h>
#include <motors.h>
#include <position_awareness.h>
#include <motor_controller.h>

#include <position_awareness.h>

#define FORWARD_TIME_BEFORE_TURN 500000
#define FORWARD_TIME_AFTER_TURN 1000000 // in us
#define TURN_TIME 600000 // in us

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

extern messagebus_t bus;

static THD_WORKING_AREA(waNavigation, 256);
static THD_FUNCTION(NavigationThd, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");
	surrounding wall_info = 0u;


    while(1){
        messagebus_topic_wait(surrounding_topic, &wall_info, sizeof(wall_info));

        if (wall_info & WALL_IN_FRONT_BIT){
        	command_motor(STOP);
        }

        else if ( (wall_info & WALL_LEFT_BIT) == 0u ){
    		 command_turn(RIGHT_TURN);
    	}
    	// only right?
    	else if ( (wall_info & WALL_RIGHT_BIT) == 0u ){
    		 command_turn(LEFT_TURN);
    	}
    	else {
    		command_motor(FORWARD_MOTION);
    	}

        chprintf((BaseSequentialStream *)&SD3, "nav s: %d \r\n", wall_info);
    }

}

/*
 * Start the thread
 */
void navigation_thread_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+4, NavigationThd, NULL);
}

void command_turn(enum motion_state direction){
	// pointer to the bus topic to write to the motors
	messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/motor_state");

	// go forward
	enum motion_state motor_state = FORWARD_MOTION; // store the info command to be published to the bus
	messagebus_topic_publish(state_topic, &motor_state, sizeof(motor_state));
	chThdSleepMicroseconds(FORWARD_TIME_BEFORE_TURN); // wait a certain time for the robot to turn

	// do the turn
	motor_state = direction; // store the info command to be published to the bus
	messagebus_topic_publish(state_topic, &motor_state, sizeof(motor_state));
	chThdSleepMicroseconds(TURN_TIME); // wait a certain time for the robot to turn

	// go forward
	motor_state = FORWARD_MOTION; // go forward after the turn
	messagebus_topic_publish(state_topic, &motor_state, sizeof(motor_state));
	chThdSleepMicroseconds(FORWARD_TIME_AFTER_TURN); // avoid imediately perfoming a 2nd turn
}

void command_motor( enum motion_state command ){
	// pointer to the bus topic to write to the motors
	messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/motor_state");
	messagebus_topic_publish(state_topic, &command, sizeof(command));
}


