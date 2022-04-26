#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <position_awareness.h>

static THD_WORKING_AREA(waMotorController, 256);
static THD_FUNCTION(MotorController, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_left = 0;
    int16_t speed_right = 0;
    int16_t lr_error = 0;

    while(1){
        time = chVTGetSystemTime();

        // get left-right error
        lr_error = get_left_right_error();

        //debug code
        speed_left = lr_error;
        speed_right = lr_error;


        // check that the speed is within the allowed limits
        if(speed_left > MOTOR_SPEED_LIMIT){speed_left = MOTOR_SPEED_LIMIT;}
        else if(speed_left < -MOTOR_SPEED_LIMIT){speed_left = -MOTOR_SPEED_LIMIT;}
        if(speed_right > MOTOR_SPEED_LIMIT){speed_right = MOTOR_SPEED_LIMIT;}
        else if(speed_right < -MOTOR_SPEED_LIMIT){speed_right = -MOTOR_SPEED_LIMIT;}


        //applies the speed from the PI regulator
		right_motor_set_speed(speed_right);
		left_motor_set_speed(speed_left);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void motor_controller_start(void){
	chThdCreateStatic(waMotorController, sizeof(waMotorController), NORMALPRIO, MotorController, NULL);
}
