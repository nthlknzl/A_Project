#ifndef POSITION_AWARENESS_H
#define POSITION_AWARENESS_H


#include <sensors/proximity.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include <chprintf.h>

typedef enum {BOTH_WALLS, NO_WALLS, ONLY_LEFT_WALL, ONLY_RIGHT_WALL} surrounding_walls_info;



void situational_awareness_thread_start(void);

/* return the difference between the left and the right (l-r) IR sensor. */
int16_t get_left_right_error( void );

// getter for detected edges
 //Returns True iff the proximity sensors detect an edge on the left side.
bool get_left_wall_presence( void );
//Returns True iff the proximity sensors detect an edge on the right side.
bool get_right_wall_presence( void );

void init_proximity_sensors( void );

#endif
