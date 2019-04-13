/*****************************************************************************-
 *                               Up.h
 *        Master Processor to Up Processor - Constants
 *****************************************************************************/
#ifndef MA_UP_H_
#define MA_UP_H_

// Messages from the Teensy to the Up-Board
enum {
  TOUP_X = 129,   // Current X position in meters
  TOUP_Y,         // Current Y position in meters
  TOUP_PITCH,     // Current Pitch for adjusting vertical row
  TOUP_HEADING,   // Current Heading
  TOUP_FPS,       // Current CO speed
  TOUP_DIST,      // Distance traveled in feet
  TOUP_RT_ENABLE, // Enable/disable route. If start, tp waits for RT_START
  TOUP_RT_START,  // Start running the route. No parameter
  TOUP_RT_NUM,    // Set the route number.
  TOUP_START_LOG, // True = start, False = end log.
  TOUP_LOG,       // String from Due to be logged.
  TOUP_KILLTP     //  Kill the process.
};

//Messages from the UpBoard to the Teensy
enum {
  FRUP_QUERY = 129, // Query for x,y, pitch and heading
  FRUP_SET_LOC_X,   // Set the X position.
  FRUP_SET_LOC_Y,   // Set the Y position.
  FRUP_SET_HEAD,    // Set the heading.
  FRUP_FPS_DIFF,    // Controls Left-right
  FRUP_FPS,         // Controls speed
  FRUP_RT_RUN,      // Turn motors on for KR & run received.
  FRUP_RT_END,      // End route.
  FRUP_RT_NUM,      // Route number
  FRUP_RUN_READY,   // Turn on Run Ready
  FRUP_MSG,         // Message to display on hand controller
  FRUP_STAT         // true = route in progress, periodic alive indicator
};

#endif /* MA_UP_H_ */
