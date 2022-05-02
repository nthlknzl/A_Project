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

#define FORWARD_TIME_AFTER_TURN 1000000 // in us
#define TURN_TIME 700000 // in us

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
static THD_FUNCTION(Navigation, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");
    surrounding_walls_info surrounding;


    while(1){
        messagebus_topic_wait(surrounding_topic, &surrounding, sizeof(surrounding)); // we use topic-read and not topicWait in order to allow the motor_controller to run more frequent than the system control thread. This is useful for the PID controller in this thread.

        switch(surrounding){
        case BOTH_WALLS: break;
        case NO_WALLS: break;
        case ONLY_LEFT_WALL: command_turn(LEFT_TURN); break;
        case ONLY_RIGHT_WALL: command_turn(RIGHT_TURN); break;
        }
        chprintf((BaseSequentialStream *)&SD3, "navigation status: %d \r\n", surrounding);
    }
}

/*
 * Start the thread
 */
void navigation_thread_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO+1, Navigation, NULL);
}

void command_turn(enum motion_state direction){
	// pointer to the bus topic to write to the motors
	messagebus_topic_t *state_topic = messagebus_find_topic_blocking(&bus, "/motor_state");
	enum motion_state motor_state = direction; // store the info command to be published to the bus
	messagebus_topic_publish(state_topic, &motor_state, sizeof(motor_state));
	chThdSleepMicroseconds(TURN_TIME); // wait a certain time for the robot to turn
	motor_state = FORWARD_MOTION; // go forward after the turn
	messagebus_topic_publish(state_topic, &motor_state, sizeof(motor_state));
	chThdSleepMicroseconds(FORWARD_TIME_AFTER_TURN); // avoid imediately perfoming a 2nd turn
}

