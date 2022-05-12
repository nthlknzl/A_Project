/* Requirements
 * In order to execure functions from this file one must priviously call the following functions
 * - proximity_start();
 * - calibrate_ir();
 * - halInit();
 * - chSysInit();
 * - messagebus_init(&bus, &bus_lock, &bus_condvar);
 */

#include <position_awareness.h>
#include <main.h>
#include <navigation.h>

//#define DEBUG

#define IR_SENSOR_LEFT_CENTER 5u
#define IR_SENSOR_RIGHT_CENTER 2u
#define IR_SENSOR_LEFT_FRONT 7u
#define IR_SENSOR_RIGHT_FRONT 0u

#define IR_SENSOR_THRESHOLD 80 // if the ir sensor measures a value lower than this threshold it assumes there's no wall on that side.
#define IR_SENSOR_FRONT_SUM_THRESHOLD 1000
// static variables
extern messagebus_t bus;


static THD_WORKING_AREA(waSituationalAwareness, 256);
static THD_FUNCTION(SituationalAwareness, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    surrounding wall_info = 0u;
	surrounding wall_info_published = 0u;

    while(1){
        time = chVTGetSystemTime();

        int debug_left_value = get_prox(IR_SENSOR_LEFT_CENTER);
        int debug_right_value = get_prox(IR_SENSOR_RIGHT_CENTER);
        //chprintf((BaseSequentialStream *)&SD3, "left: %d right: %d \r\n", debug_left_value, debug_right_value);

		// read the surrounding information from the bus
		messagebus_topic_t *surrounding_topic = messagebus_find_topic(&bus, "/surrounding");
		messagebus_topic_read(surrounding_topic, &wall_info, sizeof(wall_info));
        // reset wall information to 0 without changing line information
		wall_info &= 0b11111000;


    	if(get_prox(IR_SENSOR_LEFT_CENTER) > IR_SENSOR_THRESHOLD){ wall_info |= WALL_LEFT_BIT ;}
    	if(get_prox(IR_SENSOR_RIGHT_CENTER) > IR_SENSOR_THRESHOLD){ wall_info |= WALL_RIGHT_BIT ;}
    	if((get_prox(IR_SENSOR_LEFT_FRONT) + get_prox(IR_SENSOR_RIGHT_FRONT)) > IR_SENSOR_FRONT_SUM_THRESHOLD){ wall_info |= WALL_IN_FRONT_BIT ;}


    	// see if the situation has changed and publish if yes
    	if(wall_info != wall_info_published){
    		wall_info_published = wall_info;
			// write the wall information to the bus
			messagebus_topic_t *surrounding_topic = messagebus_find_topic(&bus, "/surrounding");
			//messagebus_topic_t *surrounding_topic = messagebus_find_topic(&bus, "/motor_state");
			messagebus_topic_publish(surrounding_topic, &wall_info, sizeof(wall_info));
    	}

        //20Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}

void situational_awareness_thread_start(void){
	chThdCreateStatic(waSituationalAwareness, sizeof(waSituationalAwareness), NORMALPRIO+3, SituationalAwareness, NULL);
}


int16_t get_left_right_error( void ){
	// store the found walls

	// store the values of the last cycle. This is used to return a difference (error for the controller) even if there's only one wall
	static int16_t left_side_distance_ref = 100; // The initial value does not matter as long as there are two walls for the first measure cycle.
	static int16_t right_side_distance_ref = 100;

	surrounding wall_info = 0u;
	messagebus_topic_t *surrounding_topic = messagebus_find_topic(&bus, "/surrounding");
    messagebus_topic_read(surrounding_topic, &wall_info, sizeof(wall_info)); // we use topic-read and not topicWait in order to allow the motor_controller to run more frequent than the system control thread. This is useful for the PID controller in this thread.


	int16_t error = 0;

	// chprintf((BaseSequentialStream *)&SD3, "sensors; %d   %d \r\n", get_prox(IR_SENSOR_LEFT_CENTER), get_prox(IR_SENSOR_RIGHT_CENTER));

	// are there walls on both sides?
	if( (wall_info & WALL_LEFT_BIT) && (wall_info & WALL_RIGHT_BIT)){
		error = get_prox(IR_SENSOR_LEFT_CENTER)-get_prox(IR_SENSOR_RIGHT_CENTER);
	}
	// only left?
	else if ( wall_info & WALL_LEFT_BIT ){
		error = get_prox(IR_SENSOR_LEFT_CENTER) - left_side_distance_ref;	}
	// only right?
	else if ( wall_info & WALL_RIGHT_BIT ){
		error = - (get_prox(IR_SENSOR_RIGHT_CENTER) - right_side_distance_ref);
	}
	// no walls?
	else{
		error = 0;
	}

	// we limit the maximal error as the values get extremely high at close distances -> no effective control possible.
	if (error > 1000){error = 1000;}
	else if (error < -1000){error = -1000;}
	return error;

}

void init_proximity_sensors( void ){
	// start and calibrate the sensor
	proximity_start();
	calibrate_ir();
}



