#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <control.h>

//motor states
static enum motion_state state;


//detection state
enum detection_state {NOTHING_DETECTED, EDGE_DETECTED, LINE_DETECTED};
enum detection_state detection;


extern messagebus_t bus;

static THD_WORKING_AREA(waControlRobot, 256);
static THD_FUNCTION(ControlRobot, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	messagebus_topic_t *processImage_topic = messagebus_find_topic_blocking(&bus, "/processImage");

	// Declares the topic for the motor message bus.
	messagebus_topic_t motor_topic;
	MUTEX_DECL(motor_topic_lock);
	CONDVAR_DECL(motor_topic_condvar);
	messagebus_topic_init(&motor_topic, &motor_topic_lock, &motor_topic_condvar, &state, sizeof(state));
	messagebus_advertise_topic(&bus, &motor_topic, "/motor_state");


	while (1) {
		time = chVTGetSystemTime();

		switch (state)
		{
		case FORWARD_MOTION:
			//get detection state information
			messagebus_topic_read(processImage_topic, &detection, sizeof(detection));
			break;
		case STOP:
			//function to choose turn direction
			state = LEFT_TURN;
			break;
		case LEFT_TURN:
			//get detection state information
			messagebus_topic_read(processImage_topic, &detection, sizeof(detection));
			break;
		case RIGHT_TURN:
			//get detection state information
			messagebus_topic_read(processImage_topic, &detection, sizeof(detection));
			break;
		default:
			break;
		}

		if (detection == LINE_DETECTED)
		{
			state = STOP;
			//get time and speed value
			//transmit values to computer
		}

		//transmettre state info au moteur
		messagebus_topic_publish(&motor_topic, &state, sizeof(state));
	}
}
