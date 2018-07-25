/******************************************************************************
 * Up-Ard.h
 *
 *  Constants for communication between UpBoard and Arduino Due
 *
 *****************************************************************************/

#ifndef UP_ARD_H_
#define UP_ARD_H_


// Messages from the Due to the Up-Board
const int TOUP_X                = 130;  // Current X position in meters
const int TOUP_Y                = 131;  // Current Y position in meters
const int TOUP_PITCH            = 132;  // Current Pitch for adjusting vertical row
const int TOUP_HEADING          = 133;  // Current Heading
const int TOUP_FPS              = 134;  // Current CO speed

const int TOUP_RT_ENABLE        = 140;  // Enable/disable route. If start, tp waits for RT_START
const int TOUP_RT_START         = 141;  // Start running the route. No parameter
const int TOUP_RT_NUM           = 142;  // Set the route number.

const int TOUP_START_LOG        = 150;  // True = start, False = end log.
const int TOUP_LOG              = 151;  // String from Due to be logged.


//Messages from the UpBoard to the Due
const int FRUP_QUERY            = 129;  // Query for x,y, pitch and heading
const int FRUP_SET_LOC_X        = 130;  // Set the X position.
const int FRUP_SET_LOC_Y        = 131;  // Set the Y position.
const int FRUP_SET_HEAD         = 132;  // Set the heading.

const int FRUP_FPS_DIFF         = 140;  // Controls Left-right
const int FRUP_FPS              = 141;  // Controls speed
const int FRUP_RT_RUN           = 142;  // Turn motors on for KR & run received.
const int FRUP_RT_END           = 143;  // End route.
const int FRUP_RT_NUM           = 144;  // Route number

const int FRUP_MSG              = 150;  // Message to display on hand controller
const int FRUP_STAT             = 151; // true = message in progress, periodic alive indicator

#endif /* UP_ARD_H_ */

