const unsigned int NO_DEBUG = 0x4242;  // If received, not displayed.

const int SEND_MESSAGE     = 129;
const int SEND_FPS         = 130;
const int SEND_PITCH       = 131;
const int SEND_HEADING     = 132;
const int SEND_SONAR       = 133;
const int SEND_ROUTE_STEP  = 134;
const int SEND_DUMP_DATA   = 135;
const int SEND_STATE       = 136;
const int SEND_BATT        = 137;
const int SEND_MODE        = 138;
const int SEND_VALSET      = 139;

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
const int RCV_HOLD_FPS     = 146;
const int RCV_GYRO_STEER   = 147;

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
//
//// Status bits in the tpState byte
//const int TP_STATE_RUN_READY       = 0B00000001;  // Ready.  Reflects the RUN command.
//const int TP_STATE_RUNNING         = 0B00000010;  // Motor running: is READY, UPRIGHT & ONGROUND
//const int TP_STATE_UPRIGHT         = 0B00000100;  // Status: tp is upright (not tipped over).
//const int TP_STATE_ON_GROUND       = 0B00001000;  // Status: pressure sensor indicates standing.
//const int TP_STATE_HC_ACTIVE       = 0B00010000;  // Status: Hand Controller connected
//const int TP_STATE_PC_ACTIVE       = 0B00100000;  // Status: PC connected
//const int TP_STATE_ROUTE           = 0B01000000;  // Status: Route in progress
//const int TP_STATE_DUMPING         = 0B10000000;  // Dumping in either direction

// value sets
const int VAL_SET_A           = 0;
const int VAL_SET_B           = 1;
const int VAL_SET_C           = 2;

// XBee MY addresses
const int XBEE_TWOPOTATOE =  0x7770;
const int XBEE_PC =          0x7771;
const int XBEE_HC =          0x7772;
const int XBEE_BROADCAST =   0xFFFF;

// XBee data packet bytes. Constant indicates positon i byte array.

	// Message\ sent by TP - byte position
const int TP_SEND_FLAG =             0;  // 1-byte, Flag and packet type
const int TP_SEND_VALUE =            1;  // 2-byte, value
const int TP_SEND_SONAR =            3;  // Sonar distance
const int TP_SEND_HEADING =          5;  // Yaw/Bearing
const int TP_SEND_END =              7;  // offset after last value	
	
	// Flag byte in TP_SEND_XXX
const int TP_SEND_FLAG_PITCH =      0;  // 1-byte, Flag and packet type
const int TP_SEND_FLAG_SPEED =      1;  
const int TP_SEND_FLAG_MODE =       2; 
const int TP_SEND_FLAG_STATE =      3;
const int TP_SEND_FLAG_BATT =       4;
const int TP_SEND_FLAG_VALSET =     7;
const int TP_SEND_FLAG_DEBUG =      8;	
const int TP_SEND_FLAG_DUMP =      25;

// Messages received by TwoPotatoe
const int TP_RCV_MSG_TYPE =        0;  // 1-byte message type or packet type
const int TP_RCV_MSG_VAL =         1;  // 2-byte message value
const int TP_RCV_X =               3;  // 1-byte x joystick
const int TP_RCV_Y =               4;  // 1-byte y joystick
const int TP_RCV_MAX =             5;

// Values that can go in the TP_RCV_MSG_TYPE byte
const int TP_RCV_MSG_NULL =        0;  // no message
const int TP_RCV_MSG_T_VAL =       1;  // 
const int TP_RCV_MSG_U_VAL =       2;  // 
const int TP_RCV_MSG_V_VAL =       3;  // 
const int TP_RCV_MSG_W_VAL =       4;  // 
const int TP_RCV_MSG_X_VAL =       5;  // 
const int TP_RCV_MSG_Y_VAL =       6;  // 
const int TP_RCV_MSG_Z_VAL =       7;  // 
const int TP_RCV_MSG_BLUE =        8;  // 
const int TP_RCV_MSG_HOME =        9;  // 
const int TP_RCV_MSG_LIGHTS =     10;  // 1st 3 bits of val
const int TP_RCV_MSG_M_MODE =     11;  // motor mode 
const int TP_RCV_MSG_ROUTE_ES =   12;  // Route, end stand
const int TP_RCV_MSG_RECORD =     13;  // Record values
const int TP_RCV_MSG_ROTATE =     14;    
const int TP_RCV_MSG_START_PW =   15;  // Run loaded pulse sequence.
const int TP_RCV_MSG_RESET =      16;  // 
const int TP_RCV_MSG_MODE =       17;  //
const int TP_RCV_MSG_VALSET =     18;  // 
const int TP_RCV_MSG_RUN_READY =  19;  // Run/Idle
const int TP_RCV_MSG_BLOCK  =     20;  // Block data, stop transmitting
const int TP_RCV_MSG_ROUTE =      21;  // Run/Halt Route
const int TP_RCV_MSG_DSTART =     22;  //
const int TP_RCV_MSG_RESET_NAV =  23;  // Reset all navigation values.

// Block types.  Must be non-overlapping with TP_RCV_MSG_xxx
const int TP_BLOCK_NULL     =    100;  // Must be greater than this
const int TP_BLOCK_ROUTE    =    101;
const int TP_BLOCK_PULSE    =    102;

// Character definitions for route actions

