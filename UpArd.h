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
const int TOUP_Y              = 131;  // Current Y position in meters
const int TOUP_PITCH            = 132;  // Current Pitch for adjusting vertical row
const int TOUP_HEADING          = 133;   // Current Heading

const int TOUP_RT_ENABLE        = 140;  // Enable current route # waiting for TOUP_RT_START
const int TOUP_RT_START         = 141;  // Start running the route.
const int TOUP_RT_NUM           = 142;  // Set the route number.

const int TOUP_START_LOG        = 150;  // True = start, False = end log.
const int TOUP_LOG              = 151;  // String from Due to be logged.


//Messages from the UpBoard to the Due
const int FRUP_QUERY            = 129;  // Query for x,y, pitch and heading
const int FRUP_SET_X      = 130;  // Set the X position.
const int FRUP_SET_Y      = 131;  // Set the Y position.
const int FRUP_SET_HEAD     = 132;  // Set the heading.

const int FRUP_GO_X       = 140;  // Controls Left-right
const int FRUP_GO_Y       = 141;  // Controls speed

const int FRUP_MSG        = 150;  // Message to display on hand controller

#endif /* UP_ARD_H_ */

