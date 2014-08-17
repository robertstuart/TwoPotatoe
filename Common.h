const unsigned int NO_DEBUG = 0x4242;  // If received, not displayed.


// Modes of operation
const int MODE_PID1           = 0;  // Original pid on board
const int MODE_PWM_SPEED      = 1;  // pw values sent from controller
const int MODE_DRIVE          = 2;  // sequences batch-loaded from controller
const int MODE_TP4            = 3;  // Motor speed controlled from interrupts
const int MODE_TP_SPEED       = 4;  // on-board motor speed
const int MODE_TP_SEQUENCE    = 5;  // sequences batch-loaded from controller
const int MODE_PULSE_SEQUENCE = 6;  // motor test
const int MODE_PULSE          = 7;
const int MODE_TP5            = 8;
const int MODE_TP6xx          = 9;
const int MODE_TP7            = 10;
const int BLOCK_DATA          = 100; 

// Status bits in the tpState byte
const int TP_STATE_RUN_READY       = 0B00000001;  // Ready.  Reflects the RUN command.
const int TP_STATE_RUNNING         = 0B00000010;  // Motor running: is READY, UPRIGHT & ONGROUND
const int TP_STATE_UPRIGHT         = 0B00000100;  // Status: tp is upright (not tipped over).
const int TP_STATE_ON_GROUND       = 0B00001000;  // Status: pressure sensor indicates standing.
//const int TP_STATE_DATA            = 0B00010000;  // TP is streaming data
const int TP_STATE_HC_ACTIVE       = 0B00100000;  // Status: Hand Controller connected
const int TP_STATE_PC_ACTIVE       = 0B01000000;  // Status: PC connected
const int TP_STATE_ROUTE           = 0B10000000;  // Route in progress.
/**************************************************************************/


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

// Messages sent by TP
const int TP_SEND_MODE_STATUS =    0;  // 1-byte, operating mode and packet type
const int TP_SEND_STATE_STATUS =   1;  // 1-byte, Status bits
const int TP_SEND_VALSET_STATUS =  2;  // 1-byte, from VAL_SET_XXX
const int TP_SEND_BATTERY =        3;  // 2-byte, battery volt * 100
const int TP_SEND_DEBUG =          5;  // 2-byte debug value
const int TP_SEND_MSG_ACK =        7;  // 1-byte ack of TP_RCV_MSG_TYPE
const int TP_SEND_MSG_ACKVAL =     8;  // 2-byte ack of TP_RCV_MSG_VAL
const int TP_SEND_END =           10;  // length of packet

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
const int TP_RCV_MSG_POWER =       8;  // 
const int TP_RCV_MSG_HOME =        9;  // 
const int TP_RCV_MSG_LIGHTS =     10;  // 1st 3 bits of val
const int TP_RCV_MSG_ROTATE =     14;    
const int TP_RCV_CMD_START_PW =   15;  // Run loaded pulse sequence.
const int TP_RCV_MSG_RESET =      16;  // 
const int TP_RCV_MSG_MODE =       17;  //
const int TP_RCV_MSG_VALSET =     18;  // 
const int TP_RCV_MSG_RUN_READY =  19;  // Run/Idle
const int TP_RCV_MSG_BLOCK  =     20;  //  Block data, stop transmitting
const int TP_RCV_MSG_DSTART =     22;  //

// Block types.  Must be non-overlapping with TP_RCV_MSG_xxx
const int TP_BLOCK_NULL     =    100;  // Must be greater than this
const int TP_BLOCK_ROUTE    =    101;
const int TP_BLOCK_PULSE    =    102;

// Character definitions for route actions

