const unsigned int NO_DEBUG = 0x4242;  // If received, not displayed.

const int XBEE_2P_SH         = 0x13A200;    // TwoPotatoe Serial
const int XBEE_2P_SL         = 0x409BD79F;
const int XBEE_3P_SH         = 0x13A200;    // ThreePotatoe Serial
const int XBEE_3P_SL         = 0x409FEBCF;
const int XBEE_C1_SH         = 0x13A200;    // Controller1 Serial
const int XBEE_C1_SL         = 0x409FEBF8;

const int XBEE_DEST_2P       = 0;           // TwoPotatoe destination
const int XBEE_DEST_3P       = 1;           // ThreePotatoe destination
const int XBEE_DEST_C1       = 3;           // Controller 1 destination

const int SEND_MESSAGE     = 129;
const int SEND_FPS         = 130;
const int SEND_PITCH       = 131;
const int SEND_HEADING     = 132;
const int SEND_SONAR_R     = 133;
const int SEND_ROUTE_STEP  = 134;
const int SEND_DUMP_DATA   = 135;
const int SEND_STATE       = 136;
const int SEND_BATT_A      = 137;
const int SEND_BATT_B      = 138;
const int SEND_MODE        = 139;
const int SEND_VALSET      = 140;
const int SEND_ROUTE_NAME  = 141;
const int SEND_X           = 142;
const int SEND_Y           = 143;
//const int SEND_SONAR_F     = 144;
const int SEND_SONAR_L     = 145;
const int SEND_DUMP_TICKS  = 146;
const int SEND_XPOS        = 190;
const int SEND_YPOS        = 191;

const int RCV_JOYX         = 129;
const int RCV_JOYY         = 130;
const int RCV_RUN          = 131;
const int RCV_LIGHTS       = 132;
const int RCV_ROUTE        = 133;
const int RCV_ROUTE_ES     = 134;
const int RCV_DUMP_START   = 135;
const int RCV_T            = 136;
const int RCV_U            = 137;
const int RCV_V            = 138;
const int RCV_W            = 139;
const int RCV_X            = 140;
const int RCV_Y            = 141;
const int RCV_Z            = 142;
const int RCV_RESET_NAV    = 143;
const int RCV_ROTATE       = 144;
const int RCV_HOLD_HEADING = 145;
const int RCV_SPIN         = 146;
const int RCV_XXXXXXXXX1   = 147; // unused
const int RCV_SET_ROUTE    = 148; // 0 to decrease, 1 to increase
const int RCV_ROUTE_DATA   = 149; 
const int RCV_XXXX_ROUTE   = 150;
const int RCV_DELETE_ROUTE = 151;
const int RCV_MODE         = 152;
const int RCV_STAND        = 153;
const int RCV_SONAR_R      = 154; // Right sonar on/off
const int RCV_SONAR_F      = 155; 
const int RCV_SONAR_L      = 156; 
const int RCV_DUMP_TICKS   = 157; 

const int SEND_RCV_TERM    =   0;

// Modes of operation
const int MODE_XXXXXXXX       = 0;  // 
const int MODE_PWM_SPEED      = 1;  // pw values sent from controller
const int MODE_DRIVE          = 2;  // sequences batch-loaded from controller
const int MODE_TP4            = 3;  // Motor speed controlled from interrupts
const int MODE_TP_SPEED       = 4;  // on-board motor speed
const int MODE_TP_SEQUENCE    = 5;  // sequences batch-loaded from controller
const int MODE_PULSE_SEQUENCE = 6;  // motor test
const int MODE_PULSE          = 7;
const int MODE_TP5            = 8;
const int MODE_TP6            = 9;
const int MODE_TP7            = 10;
const int BLOCK_DATA          = 100; 

// Motor Modes
const int MM_DRIVE_BRAKE = 0;
const int MM_DRIVE_COAST = 1;

// value sets
const int VAL_SET_A           = 0;
const int VAL_SET_B           = 1; 
const int VAL_SET_C           = 2;
