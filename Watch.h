/*****************************************************************************-
 *                        Watch.h
 *    Constants shared between TwoPotatoe and TwoPotatoe-watch
 *****************************************************************************/

// These are represented in the blinkArray in Lights
const int BLINK_OFF = 0; // Off
const int BLINK_SF =  1; // Slow flash
const int BLINK_FF =  2; // Fast flash
const int BLINK_SB =  3; // Slow blink
const int BLINK_ON =  4; // On

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

const char MUSIC_UP = 0;
const char MUSIC_UP2 = 1;
const char MUSIC_UP3 = 2; 
const char MUSIC_WARBLE = 3;
const char MUSIC_FUGUE2 = 4;
const char MUSIC_FUGUE = 5;
const char MUSIC_SCALE = 6;
const char MUSIC_BACH = 7;
const char MUSIC_DN = 8;
