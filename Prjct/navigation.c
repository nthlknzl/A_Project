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
#define FORWARD_TIME_AFTER_TURN 700000 // in us
#define TURN_TIME 600000 // in us
#define AFTER_TURN_FORWARD_DELAY_TICKS 40// in 25ms, must be <255
#define SLEEP_TIME_AFTER_COMMAND 500000

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

static motion_state motor_state = STOP;


static THD_WORKING_AREA(waNavigation, 256);
static THD_FUNCTION(NavigationThd, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");
	surrounding wall_info = 0u;

    while(1){
        messagebus_topic_wait(surrounding_topic, &wall_info, sizeof(wall_info));
        // the motor modes only need to be updated if there is new information

        // the following lines decide what to do based on the walls and lines detected. This can be modified to
        // change the robots behaviour
        if (wall_info & WALL_IN_FRONT_BIT){
        	command_motor(STOP);
        }
        else if (wall_info & FLOOR_LINE_IN_FRONT){
        	command_motor(STOP);
        }
        else if ( (wall_info & WALL_LEFT_BIT) == 0u ){
        	command_turn(RIGHT_TURN);
    	}
    	else if ( (wall_info & WALL_RIGHT_BIT) == 0u ){
    		command_turn(LEFT_TURN);
    	}
    	else {
    		command_motor(FORWARD_MOTION);
    	}
    }

}

/*
 * Start the thread
 */
void navigation_thread_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+4, NavigationThd, NULL);
}


/*
 * command_turn:
 * This functions performas a left or right turn by
 * 		1. drive forward (to hit the center of the street)
 * 		2. turn on the spot
 * 		3. drive forward (to go back into the street)
 */
void command_turn(motion_state direction){
	// pointer to the bus topic to write to the motors

	// 1. go forward
	command_motor(FORWARD_MOTION);
	chThdSleepMicroseconds(FORWARD_TIME_BEFORE_TURN); // wait a certain time for the robot to turn

	// 2. do the turn
	command_motor(direction);
	chThdSleepMicroseconds(TURN_TIME); // wait a certain time for the robot to turn

	// 3. go forward
	command_motor(FORWARD_MOTION);

	systime_t time;
	messagebus_topic_t *surrounding_topic;
	surrounding wall_info = 0u;
	// this for loop ensures that the e-puck does not blindly drives forward after the turn but detects walls and stops if needed.
	// The robot checks frequently if there is a wall in front of him and stops if needed.
	for(uint8_t poll_nr = 0; poll_nr<AFTER_TURN_FORWARD_DELAY_TICKS; poll_nr++ ){
        time = chVTGetSystemTime();
        // read surounding information
        surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");
        messagebus_topic_read(surrounding_topic, &wall_info, sizeof(wall_info));
        if (wall_info & WALL_IN_FRONT_BIT){
        	command_motor(STOP);
        	return;
        }
		chThdSleepUntilWindowed(time, time + MS2ST(25));
	}
	//chThdSleepMicroseconds(FORWARD_TIME_AFTER_TURN); // avoid imediately perfoming a 2nd turn
}

void command_motor(motion_state command ){
	// pointer to the bus topic to write to the motors
	motor_state = command;
	messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/motor_state");
	messagebus_topic_publish(state_topic, &command, sizeof(command));

	//chThdSleepMicroseconds(SLEEP_TIME_AFTER_COMMAND);

}


