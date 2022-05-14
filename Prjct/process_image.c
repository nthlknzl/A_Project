#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>
#include <navigation.h>

extern messagebus_t bus;

//constants for image capture
#define IMAGE_BUFFER_SIZE		480	//nb of pixels in one column
#define IMAGE_BUFFER_SIZE_SEND	640	//the python file works with a buffer size of 640
#define IMAGE_COLUMN_SIZE		2 	//2 columns
#define COL_START				320
#define LIN_START				0

//constants for general image procession
#define AVE_NB 	30
#define FIRST_COL 0

//constants for line detection
#define ED_STEP 				50
#define CHECK_STEP 				40
#define	EDGE_HEIGHT_MIN 		50

//for debugging
//#define DEBUG_IMAGE

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


static void column_extraction(uint8_t *image_all_colors, uint8_t *img_buff_ptr, uint16_t col);
static void color_extraction_red(uint8_t *image, uint8_t *img_buff_ptr);
static void cumulative_moving_average(uint8_t *buffer, uint8_t nb_val_for_ave);
static uint8_t line_detection(uint8_t *img_values, surrounding surrounding_info);
static bool falling_edge_detection(uint8_t *img_values, uint16_t i);
static bool rising_edge_detection(uint8_t *img_values, uint16_t i);

//debug function: sending image to computer for verification
static void send_to_computer(uint8_t *image);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the column 320 + 321 (minimum 2 columns because reasons)
	po8030_advanced_config(FORMAT_RGB565, COL_START, LIN_START, IMAGE_COLUMN_SIZE,
			IMAGE_BUFFER_SIZE, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while (1) {
		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
	}
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	messagebus_topic_t *surrounding_topic = messagebus_find_topic_blocking(&bus, "/surrounding");
	surrounding surrounding_info = 0u;

	static bool surrounding_info_published = FALSE;

	uint8_t *img_buff_ptr;
	static uint8_t image_all_colors[2*IMAGE_BUFFER_SIZE] = { 0 };
	static uint8_t image[IMAGE_BUFFER_SIZE] = { 0 };

	while (1) {
		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//image processing
		column_extraction(image_all_colors, img_buff_ptr, FIRST_COL);
		color_extraction_red(image, image_all_colors);
		cumulative_moving_average(image, AVE_NB);

		//read the surrounding information from the bus
		messagebus_topic_read(surrounding_topic, &surrounding_info, sizeof(surrounding_info));

		//line detection
		surrounding_info = line_detection(image, surrounding_info);

		//publish surrounding_info after set of line_detection bit
		if (surrounding_info & FLOOR_LINE_IN_FRONT) {
			// Publishes it on the bus.
			messagebus_topic_publish(surrounding_topic, &surrounding_info, sizeof(surrounding_info));
			surrounding_info_published = TRUE;
		}

		/* if the surrounding info has been published after line detection, and no line is detected anymore,
		 * it must be published that there isn't a line anymore.
		 */
		if (surrounding_info_published && (surrounding_info & FLOOR_LINE_IN_FRONT)== 0u) {
			// Publishes it on the bus.
			messagebus_topic_publish(surrounding_topic, &surrounding_info, sizeof(surrounding_info));
			surrounding_info_published = FALSE;
		}

#ifdef DEBUG_IMAGE 	//sending image to computer for verification
		send_to_computer(image);
#endif
	}
}

/* Extracts one column
 *
 *  input: column number (of the captured image!)
 */
static void column_extraction(uint8_t *image_all_colors, uint8_t *img_buff_ptr, uint16_t col){
	for (uint16_t i = 0; i < (IMAGE_BUFFER_SIZE); i += 1) {
		//2 bytes per pixel
		image_all_colors[2*i] = (uint8_t) img_buff_ptr[(col+IMAGE_COLUMN_SIZE*i)*2];
		image_all_colors[2*i+1] = (uint8_t) img_buff_ptr[(col+IMAGE_COLUMN_SIZE*i)*2+1];
	}
}


/* Extracts the bits for red color
 *
 *  red corresponds to the first 5bits of the first byte
 *  takes nothing from the second byte
 *  the color information is left at the left to get higher values
 */
static void color_extraction_red(uint8_t *image, uint8_t *image_all_colors){
	for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
		image[i / 2] = (uint8_t) (image_all_colors[i] & 0xF8);
	}
}


/* Cumulative Moving Average
 *
 *  calculate average over nb_val_for_ave numbers
 *  the nb_val_for_ave/2 elements on the right and and the left of the buffer remain unchanged
 */
static void cumulative_moving_average(uint8_t *buffer, uint8_t nb_val_for_ave) {
	uint16_t ave_sum = 0;
	for (int i = 0; i < (IMAGE_BUFFER_SIZE - nb_val_for_ave); i++) {
		for (int j = 0; j < nb_val_for_ave; j++) {
			ave_sum += buffer[i + j];
		}
		buffer[i + nb_val_for_ave/2] = ave_sum / nb_val_for_ave;
		ave_sum = 0;
	}
}

/* Detects a colored line on white background
 *
 *  detection of white parts give high RGB values, colored parts low values.
 *	so we detect a rapid decrease in the values followed by a rapid increase
 */
static uint8_t line_detection(uint8_t *img_values, surrounding surrounding_info) {
    static systime_t time_falling_edge = 0;
    static systime_t time_rising_edge = 0;

    static bool falling_edge_detected = FALSE;

	//reset line detection bit to 0
	surrounding_info &= 0b01111111;

	for (uint16_t i = 0; i < (IMAGE_BUFFER_SIZE - ED_STEP - CHECK_STEP); i++) {
		//falling edge
		if (img_values[i] > img_values[i + ED_STEP]) {
			if (falling_edge_detection(img_values, i)) {
				falling_edge_detected = TRUE;
				time_falling_edge = chVTGetSystemTime();
			}
		//rising edge
		} else {
			if (rising_edge_detection(img_values, i) && falling_edge_detected) {
				time_rising_edge = chVTGetSystemTime();
				//line only if rising edge happened less than 1s after falling edge
				if(ST2S(time_rising_edge-time_falling_edge) < 1) {
					//set line detection bit to 1
					surrounding_info |= FLOOR_LINE_IN_FRONT;
					//reset falling edge information
					falling_edge_detected = FALSE;
				}
			}
		}
	}
	return surrounding_info;
}

static bool falling_edge_detection(uint8_t *img_values, uint16_t i) {
	//no edge if height difference is too small
	if ((img_values[i] - img_values[i + ED_STEP]) < EDGE_HEIGHT_MIN) {
		return FALSE;
	}
	//no edge if there's an edge in the other direction within less than CHECK_STEP pixels
	else if ((img_values[i + ED_STEP + CHECK_STEP] > img_values[i + ED_STEP]) &&
			 (img_values[i+ED_STEP+CHECK_STEP]-img_values[i+ED_STEP]) > EDGE_HEIGHT_MIN) {
		return FALSE;
	}
	//edge detected
	else {
		return TRUE;
	}
}

static bool rising_edge_detection(uint8_t *img_values, uint16_t i) {
	//no edge if height difference is too small
	if ((img_values[i + ED_STEP] - img_values[i]) < EDGE_HEIGHT_MIN) {
		return FALSE;
	}
	//no edge if there's an edge in the other direction within less than CHECK_STEP pixels
	else if ((img_values[i + ED_STEP + CHECK_STEP] < img_values[i + ED_STEP]) &&
			 (img_values[i+ED_STEP] - img_values[i+ED_STEP+CHECK_STEP]) > EDGE_HEIGHT_MIN) {
		return FALSE;
	}
	//edge detected
	else {
		return TRUE;
	}
}


//debug function: sending image to computer for verification
static void send_to_computer(uint8_t *image) {
	static uint8_t image_send[IMAGE_BUFFER_SIZE_SEND] = { 0 };
	static bool send = true;

	for (uint16_t i = 0; i < (IMAGE_BUFFER_SIZE-AVE_NB); i++) {
		image_send[i] = image[i];
	}

	if (send) {
		SendUint8ToComputer(image_send, IMAGE_BUFFER_SIZE_SEND);
	}

	//invert the bool
	send = !send;
}

void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+2,
			ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO,
			CaptureImage, NULL);
}
