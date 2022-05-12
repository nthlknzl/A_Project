#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/*
#define NOTHING_DETECTED 		0
#define RISING_EDGE_DETECTED 	1
#define FALLING_EDGE_DETECTED 	1
#define	LINE_DETECTED 			2
*/

//constants for process_image.c: image capture
#define IMAGE_BUFFER_SIZE		480// 480 pixels in one column
#define IMAGE_BUFFER_SIZE_SEND	640// 480 pixels in one column
#define IMAGE_COLUMN_SIZE		2 // 2 pixels per column
#define COL_START				320
#define LIN_START				0

//constants for process_image.c: average calculation
#define AVE_NB 	30

//constants for process_image.c: edge detection
#define ED_STEP 				40
#define CHECK_STEP 				50
#define	EDGE_HEIGHT_MIN 		60

// wait after line detection
#define WAIT_MS 1000

//other constants for the differents parts of the project -> copied from TP4
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

//for debugging----------------------------------------------------------------------------------------------
//#define DEBUG
#define DEBUG_IMAGE
//#define DEBUG_LINE_DETECTION

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
