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

//#define DEBUG

#define IR_SENSOR_LEFT_CENTER 5u
#define IR_SENSOR_RIGHT_CENTER 2u
#define IR_SENSOR_LEFT_FRONT 7u
#define IR_SENSOR_RIGHT_FRONT 0u

#define IR_SENSOR_THRESHOLD  30 // if the ir sensor measures a value lower than this threshold it assumes there's no wall on that side.
#define IR_SENSOR_FRONT_SUM_THRESHOLD 1000
// static variables
extern messagebus_t bus;


int16_t get_left_right_error( void ){
	// store the found walls

	// store the values of the last cycle. This is used to return a difference (error for the controller) even if there's only one wall
	static int16_t left_side_distance_ref = 100; // The initial value does not matter as long as there are two walls for the first measure cycle.
	static int16_t right_side_distance_ref = 100;

	int16_t error = 0;

	chprintf((BaseSequentialStream *)&SD3, "sensors; %d   %d \r\n", get_prox(IR_SENSOR_LEFT_CENTER), get_prox(IR_SENSOR_RIGHT_CENTER));

	surrounding wall_info = 0u;

	if(get_prox(IR_SENSOR_LEFT_CENTER) > IR_SENSOR_THRESHOLD){ wall_info |= WALL_LEFT_BIT ;}
	if(get_prox(IR_SENSOR_RIGHT_CENTER) > IR_SENSOR_THRESHOLD){ wall_info |= WALL_RIGHT_BIT ;}
	if((get_prox(IR_SENSOR_LEFT_FRONT) + get_prox(IR_SENSOR_RIGHT_FRONT)) > IR_SENSOR_FRONT_SUM_THRESHOLD){ wall_info |= WALL_IN_FRONT_BIT ;}

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

	chprintf((BaseSequentialStream *)&SD3, "wall %d \r\n", wall_info);

	// write the wall information to the bus
    messagebus_topic_t *surrounding_topic = messagebus_find_topic(&bus, "/surrounding");
    //messagebus_topic_t *surrounding_topic = messagebus_find_topic(&bus, "/motor_state");
	messagebus_topic_publish(surrounding_topic, &wall_info, sizeof(wall_info));



	/*
	if((get_prox(IR_SENSOR_LEFT_CENTER) > IR_SENSOR_THRESHOLD) && (get_prox(IR_SENSOR_RIGHT_CENTER) > IR_SENSOR_THRESHOLD)){
		// there are walls on both sides  -> error = left-right
		error = get_prox(IR_SENSOR_LEFT_CENTER)-get_prox(IR_SENSOR_RIGHT_CENTER);
		surrounding = BOTH_WALLS;
		#ifdef DEBUG
			chprintf((BaseSequentialStream *)&SD3, "mode |o|; error: %d \r\n", error);
		#endif
	} else if (get_prox(IR_SENSOR_LEFT_CENTER) > IR_SENSOR_THRESHOLD){
		// there is only a wall on the left side -> keep a constant distance to that wall
		error = get_prox(IR_SENSOR_LEFT_CENTER) - left_side_distance_ref;
		surrounding = ONLY_LEFT_WALL;
		#ifdef DEBUG
			chprintf((BaseSequentialStream *)&SD3, "mode |o ; error: %d \r\n", error);
		#endif
	} else if (get_prox(IR_SENSOR_RIGHT_CENTER) > IR_SENSOR_THRESHOLD){
		// there is only a wall on the right side -> keep a constant distance to that wall
		error = get_prox(IR_SENSOR_RIGHT_CENTER) - right_side_distance_ref;
		surrounding = ONLY_RIGHT_WALL;
		#ifdef DEBUG
			chprintf((BaseSequentialStream *)&SD3, "mode  o|; error: %d \r\n", error);
		#endif
	} else {
		// there is no wall -> return a zero error.
		error = 0;
		surrounding = NO_WALLS;
	}
*/


	// we limit the maximal error as the values get extremely high at close distances -> no effective control possible.
	if (error > 500){error = 500;}
	else if (error < -500){error = -500;}
	return error;

}

void init_proximity_sensors( void ){
	// start and calibrate the sensor
	proximity_start();
	calibrate_ir();
}



