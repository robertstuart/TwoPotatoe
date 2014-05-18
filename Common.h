const unsigned int NO_DEBUG = 0x4242;  // If received, not displayed.


// Modes of operation
const int MODE_PID1           = 0;  // Original pid on board
const int MODE_PWM_SPEED      = 1;  // pw values sent from controller
const int MODE_DRIVE          = 2;  // sequences batch-loaded from controller
const int MODE_TP4            = 3;  // Motor speed controlled from interrupts
const int MODE_TP_SPEED       = 4;  // on-board motor speed
const int MODE_TP_SEQUENCE    = 5;  // sequences batch-loaded from controller
const int MODE_MT_SEQUNCE     = 6;  // motor test
const int MODE_IMU            = 7;
const int MODE_TP5            = 8;

// Status bits in the tpState byte
const int TP_STATE_RUN_READY       = 0B00000001;  // Ready.  Reflects the RUN command.
const int TP_STATE_RUNNING         = 0B00000010;  // Motor running: is READY, UPRIGHT & ONGROUND
const int TP_STATE_UPRIGHT         = 0B00000100;  // Status: tp is upright (not tipped over).
const int TP_STATE_ON_GROUND       = 0B00001000;  // Status: pressure sensor indicates standing.
const int TP_STATE_STREAMING       = 0B00010000;  // TP is streaming data
const int TP_STATE_HC_ACTIVE       = 0B00100000;  // Status: Hand Controller connected
const int TP_STATE_PC_ACTIVE       = 0B01000000;  // Status: PC connected
const int TP_STATE_DUMPING         = 0B10000000;  // Collecting or dumping data.

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
const int TP_SEND_A_VAL =         10;  // 4-byte
const int TP_SEND_B_VAL =         14;  // 4-byte
const int TP_SEND_C_VAL =         18;  // 2-byte
const int TP_SEND_D_VAL =         20;  // 2-byte
const int TP_SEND_E_VAL =         22;  // 2-byte
const int TP_SEND_F_VAL =         24;  // 2-byte
const int TP_SEND_G_VAL =         26;  // 2-byte
const int TP_SEND_H_VAL =         28;  // 2-byte
const int TP_SEND_I_VAL =         30;  // 2-byte
const int TP_SEND_J_VAL =         32;  // 2-byte
const int TP_SEND_MAX =           34;

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
const int TP_RCV_MSG_STREAM =     11;  // no message
const int TP_RCV_MSG_RATE =       12;  // 0 = 20/sec, 1 = 100/sec
const int TP_RCV_MSG_COLLECT =    13;  // 1 = on, 0 = off;
const int TP_RCV_MSG_ROTATE =     14;    
const int TP_RCV_MSG_START =      15;  // Run loaded sequence.
const int TP_RCV_MSG_RESET =      16;  // 
const int TP_RCV_MSG_MODE =       17;  //
const int TP_RCV_MSG_VALSET =     18;  // 
const int TP_RCV_MSG_RUN    =     19;  // Run/Idle

