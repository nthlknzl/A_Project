#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <control.h>

//motor states
enum motion_state {FORWARD_MOTION, STOP, LEFT_TURN, RIGHT_TURN};
static enum motion_state state;


//detection state
enum detection_state {NOTHING_DETECTED, EDGE_DETECTED, LINE_DETECTED, CIRCLE_DETECTED};
enum detection_state detection;


extern messagebus_t bus;

static THD_WORKING_AREA(waControlRobot, 256);
static THD_FUNCTION(ControlRobot, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	messagebus_topic_t *processImage_topic = messagebus_find_topic_blocking(&bus, "/processImage");


	while (1) {
		time = chVTGetSystemTime();

		//wait for detection state information
		messagebus_topic_wait(processImage_topic, &detection, sizeof(detection));

		switch (state)
		{
		case LINE_DETECTED:
			state = STOP;
			// function to choose direction
			break;
		case CIRCLE_DETECTED:
			// do sth;
			break;
		default:
			break;
		}

		// transmettre state info au moteur


		//T=10ms -> f=100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}
