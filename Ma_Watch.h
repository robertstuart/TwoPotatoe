/*****************************************************************************-
 *                        Watch.h
 *    Master processor to watchdog processor constants
 *****************************************************************************/

// These are represented in the blinkArray in Lights
const int BLINK_OFF = 0; // Off
const int BLINK_SF =  1; // Slow flash
const int BLINK_FF =  2; // Fast flash
const int BLINK_SB =  3; // Slow blink
const int BLINK_ON =  4; // On
const int BLINK_20 =  5; //
const int BLINK_40 =  6; //
const int BLINK_60 =  7; //
const int BLINK_80 =  8; //

const int LED_SW_GN = 0;
const int LED_SW_RE = 1;
const int LED_SW_BU = 2;
const int LED_SW_YE = 3;

const int SEND_WATCH     = 129;     // Watchdog message. No payload. 
const int SEND_BLINK     = 130;     // Led, Pattern.
const int SEND_BEEP      = 131;     // Pattern.
const int SEMD_XXX       = 132;

const int RCV_BATT       = 129;     // Battery voltage, float;
const int RCV_SWITCH     = 130;

enum {
  T_RHAPSODY,
  T_FUGUE,
  T_UP1,
  T_UP2,
  T_UP3, 
  T_DN,
  T_DN4,
  T_WARBLE,
  T_BACH,
  T_FUGUE2,
  T_SCALE,
  T_BROWNOUT,
  T_WELCOME,
  T_THANKYOU,
  T_BUTTONA,
  T_BUTTONB,
  T_BUTTONC,
  T_ALARM
};
