/*#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <control.h>

//motor states
static enum motion_state state;

//detection state
static enum detection_state detection;

extern messagebus_t bus;


static THD_WORKING_AREA(waReactOnDetection, 256);
static THD_FUNCTION(ReactOnDetection, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	messagebus_topic_t *surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");

	// Declares the topic for the motor message bus.
	messagebus_topic_t motor_state_topic;
	MUTEX_DECL(motor_state_topic_lock);
	CONDVAR_DECL(motor_state_topic_condvar);
	messagebus_topic_init(&motor_state_topic, &motor_state_topic_lock, &motor_state_topic_condvar, &state, sizeof(state));
	messagebus_advertise_topic(&bus, &motor_state_topic, "/motor_state");

	while (1) {
		time = chVTGetSystemTime();

		messagebus_topic_wait(surrounding_topic, &surrounding_info, sizeof(surrounding_info));
		if (detection == LINE_DETECTED)
		{
			//get time and speed value
			//calculate distance
			//transmit to computer
			state = STOP;
		}

		//transmettre state info au moteur
		messagebus_topic_publish(&motor_state_topic, &state, sizeof(state));
	}
}
*/
