#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <camera/po8030.h>
#include <position_awareness.h>

#include <pi_regulator.h>
#include <process_image.h>
#include <motor_controller.h>
#include <navigation.h>

#include <arm_math.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{
    halInit();
    chSysInit();
    //mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts the camera
    //dcmi_start();
    //po8030_start();
    //inits the motors
    motors_init();

	//starts the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	//process_image_start();

	init_proximity_sensors();

	// init messagebus
	// init the messagebus
		enum motion_state initital_state = FORWARD_MOTION;
		messagebus_topic_t motor_topic;
		MUTEX_DECL(motor_topic_lock);
		CONDVAR_DECL(motor_topic_condvar);
		messagebus_topic_init(&motor_topic, &motor_topic_lock, &motor_topic_condvar, &initital_state, sizeof(initital_state));
		messagebus_advertise_topic(&bus, &motor_topic, "/motor_state");

		// create a messagebus topic to publish information about the surrounding walls
		surrounding wall_info = 0u;
		messagebus_topic_t surrounding_topic;
		MUTEX_DECL(surrounding_topic_lock);
		CONDVAR_DECL(surrounding_topic_condvar);
		messagebus_topic_init(&surrounding_topic, &surrounding_topic_lock, &surrounding_topic_condvar, &wall_info, sizeof(wall_info));
		messagebus_advertise_topic(&bus, &surrounding_topic, "/surrounding");

	//motor_controller_test();
	situational_awareness_thread_start();
	motor_controller_start();
	navigation_thread_start();


    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);

        /*
        // test code to test teh pos awareness
        int diff = get_left_right_error();
        chThdSleepMilliseconds(500);
        chprintf((BaseSequentialStream *)&SD3, "left-right difference: %d \r\n", diff);
        // end test pos awareness
         */
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
