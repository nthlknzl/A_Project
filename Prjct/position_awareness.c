/* Requirements
 * In order to execure functions from this file one must priviously call the following functions
 * - proximity_start();
 * - calibrate_ir();
 * - halInit();
 * - chSysInit();
 * - messagebus_init(&bus, &bus_lock, &bus_condvar);
 */

#include <position_awareness.h>

#define DEBUG

#define IR_SENSOR_LEFT_CENTER 5u
#define IR_SENSOR_RIGHT_CENTER 2u
#define IR_SENSOR_LEFT_45FRONT 6u
#define IR_SENSOR_LEFT_45BACK 4u

#define IR_SENSOR_THRESHOLD  15 // if the ir sensor measures a value lower than this threshold it assumes there's no wall on that side.




int get_left_right_error( void ){
	// store the values of the last cycle. This is used to return a difference (error for the controller) even if there's only one wall
	static int left_side_distance_ref = 100; // The initial value does not matter as long as there are two walls for the first measure cycle.
	static int right_side_distance_ref = 100;

	int error = 0;

	if((get_prox(IR_SENSOR_LEFT_CENTER) > IR_SENSOR_THRESHOLD) && (get_prox(IR_SENSOR_RIGHT_CENTER) > IR_SENSOR_THRESHOLD)){
		// there are walls on both sides  -> error = left-right
		error = get_prox(IR_SENSOR_LEFT_CENTER)-get_prox(IR_SENSOR_RIGHT_CENTER);
		#ifdef DEBUG
			chprintf((BaseSequentialStream *)&SD3, "mode |o|; error: %d \r\n", error);
		#endif
	} else if (get_prox(IR_SENSOR_LEFT_CENTER) > IR_SENSOR_THRESHOLD){
		// there is only a wall on the left side -> keep a constant distance to that wall
		error = get_prox(IR_SENSOR_LEFT_CENTER) - left_side_distance_ref;
		#ifdef DEBUG
			chprintf((BaseSequentialStream *)&SD3, "mode |o ; error: %d \r\n", error);
		#endif
	} else if (get_prox(IR_SENSOR_RIGHT_CENTER) > IR_SENSOR_THRESHOLD){
		// there is only a wall on the right side -> keep a constant distance to that wall
		error = get_prox(IR_SENSOR_RIGHT_CENTER) - right_side_distance_ref;
		#ifdef DEBUG
			chprintf((BaseSequentialStream *)&SD3, "mode  o|; error: %d \r\n", error);
		#endif
	} else {
		// there is no wall -> return a zero error.
		error = 0;
	}
	return error;
}

float get_left_crossroad_center_error( void ){
	int front_left_sensor = get_prox(IR_SENSOR_LEFT_45FRONT);
	int back_left_sensor = get_prox(IR_SENSOR_LEFT_45BACK);
	float error = (float)(front_left_sensor - back_left_sensor) / (front_left_sensor + back_left_sensor);
	#ifdef DEBUG
		chprintf((BaseSequentialStream *)&SD3, "left crossroad error: %f \r\n", error);
	#endif
	return error;
}





