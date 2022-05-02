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



static THD_WORKING_AREA(waNavigation, 256);
static THD_FUNCTION(Navigation, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    messagebus_topic_t *surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");
    surrounding_walls_info surrounding;

    while(1){
        time = chVTGetSystemTime();
        messagebus_topic_wait(surrounding_topic, &surrounding, sizeof(surrounding)); // we use topic-read and not topicWait in order to allow the motor_controller to run more frequent than the system control thread. This is useful for the PID controller in this thread.

        chprintf((BaseSequentialStream *)&SD3, "navigation status: %d \r\n", surrounding);
    }
}

/*
 * Start the thread
 */
void navigation_thread_start(void){
	chThdCreateStatic(waNavigation, sizeof(waNavigation), NORMALPRIO, Navigation, NULL);
}

