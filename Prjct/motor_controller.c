#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <position_awareness.h>

#define BASE_SPEED 500

static THD_WORKING_AREA(waMotorController, 256);
static THD_FUNCTION(MotorController, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_left = 0;
    int16_t speed_right = 0;
    int16_t lr_error = 0;

    float Kp = 0.1;
    float Ki = 0.1;

    static integral_error = 0;

    while(1){
        time = chVTGetSystemTime();

        // get left-right error
        lr_error = get_left_right_error();

        // P
        speed_left += Kp * lr_error;
        speed_right -= Kp * lr_error;

        // I
        integral_error += lr_error;
        if(integral_error > BASE_SPEED/Ki){integral_error = 0.2*BASE_SPEED/Ki;}
        else if(integral_error < - BASE_SPEED/Ki){integral_error = -0.2*BASE_SPEED/Ki;}
        /*
        speed_left += Ki * integral_error;
        speed_right -= Ki * integral_error;
*/

        speed_left += BASE_SPEED;
        speed_right += BASE_SPEED;





        // check that the speed is within the allowed limits
        if(speed_left > MOTOR_SPEED_LIMIT){speed_left = MOTOR_SPEED_LIMIT;}
        else if(speed_left < -MOTOR_SPEED_LIMIT){speed_left = -MOTOR_SPEED_LIMIT;}
        if(speed_right > MOTOR_SPEED_LIMIT){speed_right = MOTOR_SPEED_LIMIT;}
        else if(speed_right < -MOTOR_SPEED_LIMIT){speed_right = -MOTOR_SPEED_LIMIT;}


        //applies the speed from the PI regulator

        right_motor_set_speed(speed_right);
        left_motor_set_speed(speed_left);

        chprintf((BaseSequentialStream *)&SD3, "left: %d \t right: %d \r\n", speed_left, speed_right);

        //20Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}

void motor_controller_start(void){
	chThdCreateStatic(waMotorController, sizeof(waMotorController), NORMALPRIO, MotorController, NULL);
}
