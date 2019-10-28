/*****************************************************************************-
 *                               Ma_Up.h
 *        Master Processor to Up Processor - Constants
 *****************************************************************************/
#ifndef MA_UP_H_
#define MA_UP_H_

const double TICKS_PER_FOOT = 2222.0; // For Losi DB XL 1/5 scale

// Messages from the Teensy to the Up-Board
enum {
  TOUP_HEADING = 129,   // Current Heading
  TOUP_FPS,             // Current CO speed
  TOUP_TICKS,           // Distance traveled in ticks
  TOUP_PITCH,           //
  TOUP_V_ACCEL,         // Acceleration on vertical axis
  TOUP_RT_ENABLE,       // Enable/disable route. If start, tp waits for RT_START
  TOUP_RT_START,        // Start running the route. No parameter
  TOUP_RT_NUM,          // true = increment, false = decrement
  TOUP_START_LOG,       // True = start, False = end log.
  TOUP_LOG,             // String from Teensy to be logged.
  TOUP_KILLTP,          //  Kill the process.
  TOUP_EVENT            // misc signal
};

//Messages from the UpBoard to the Teensy
enum {
  FRUP_STEER = 129,     // Controls speed
  FRUP_FPS,             // Controls stear left-right
  FRUP_MSG,             // Message or route to display on hand controller 
  FRUP_STAT,            // Status bits.
  FRUP_CAPITCH          // Pitch from camera
};

#endif /* MA_UP_H_ */
