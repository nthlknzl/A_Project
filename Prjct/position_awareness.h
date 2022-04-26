#ifndef POSITION_AWARENESS_H
#define POSITION_AWARENESS_H


#include <sensors/proximity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include <chprintf.h>



/* return the difference between the left and the right (l-r) IR sensor. */
int get_left_right_error( void );
float get_left_crossroad_center_error( void );
#endif
