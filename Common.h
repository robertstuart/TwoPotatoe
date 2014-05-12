const unsigned long NO_DEBUG = 0xFFFFFFFF;  // If received, not displayed.


// Modes of operation
const int MODE_PID1           = 0;  // Original pid on board
const int MODE_PWM_SPEED      = 1;  // pw values sent from controller
const int MODE_DRIVE          = 2;  // sequences batch-loaded from controller
const int MODE_TP4            = 3;  // Motor speed controlled from interrupts
const int MODE_TP_SPEED       = 4;  // on-board motor speed
const int MODE_TP_SEQUENCE    = 5;  // sequences batch-loaded from controller
const int MODE_MT_SEQUNCE     = 6;  // motor test
const int MODE_IMU            = 7;

// Status bits in the tpState byte
const int TP_STATE_RUN_READY       = 0B00000001;  // Ready.  Reflects the RUN command.
const int TP_STATE_RUNNING         = 0B00000010;  // Motor running: is READY, UPRIGHT & ONGROUND
const int TP_STATE_UPRIGHT         = 0B00000100;  // Status: tp is upright (not tipped over).
const int TP_STATE_ON_GROUND       = 0B00001000;  // Status: pressure sensor indicates standing.
const int TP_STATE_DUMPING         = 0B00010000;  // Collecting or dumping data.
const int TP_STATE_HC_ACTIVE       = 0B00100000;  // Status: Hand Controller connected
const int TP_STATE_PC_ACTIVE       = 0B01000000;  // Status: PC connected
const int TP_STATE_STREAMING       = 0B10000000;  // TP is streaming data

// Command bits in the cmdState byte
const int CMD_STATE_RUN =        0B00000001;  // Command run or be idle
const int CMD_STATE_PWR =        0B00000010;  // zero turns off battery power
const int CMD_STATE_HOME =       0B00000100;  // tells tp to stay in one spot
const int CMD_STATE_STREAM =     0B00001000;  // Stream data.
const int CMD_STATE_DUMP =       0B00010000;  // Collect data while true, then dump

// value sets
const int VAL_SET_A           = 0;
const int VAL_SET_B           = 1;
const int VAL_SET_C           = 2;

// For commands that have two states, (0x20) bit indicates the state
const int CMD_HC_SOURCE =   64; //B01000000;
const int CMD_PARAM =       32; //B00100000;  // Start for parameterized commands
const int CMD_SINGLE_FLAG = 16; //B00010000;  // Indicate true or false for single-byte commands


// ----------------- Command constants sent from TP --------------------
// Single-byte commands, 
const int CMD_QUERY =                            0;
const int CMD_SEQUENCE_END =                     1;
const int CMD_UPLOAD_START =                     2;
const int CMD_STREAM_STATUS =                    3;

// XBee MY addresses
const int XBEE_TWOPOTATOE =  0x7770;
const int XBEE_PC =          0x7771;
const int XBEE_HC =          0x7772;
const int XBEE_BROADCAST =   0xFFFF;

// XBee data packet bytes. Constant indicates positon i byte array.

// Messages sent by TP
const int TP_SEND_PACKET_TYPE =    0;  // 1-byte, type of remaining message
const int TP_SEND_STATE_STATUS =   1;  // 1-byte, Status bits
const int TP_SEND_MODE_STATUS =    2;  // 1-byte, Mode from MODE_XXX
const int TP_SEND_VALSET_STATUS =  3;  // 1-byte, from VAL_SET_XXX
const int TP_SEND_BATTERY =        4;  // 2-byte, battery volt * 100
const int TP_SEND_DEBUG =          6;  // 4-byte debug value
const int TP_SEND_MSG_ACK =       10;  // 1-byte ack of TP_RCV_MSG_TYPE
const int TP_SEND_MSG_ACKVAL =    11;  // 2-byte ack of TP_RCV_MSG_VAL
const int TP_SEND_A_VAL =         13;  // 4-byte
const int TP_SEND_B_VAL =         17;  // 4-byte
const int TP_SEND_C_VAL =         21;  // 2-byte
const int TP_SEND_D_VAL =         23;  // 2-byte
const int TP_SEND_E_VAL =         25;  // 2-byte
const int TP_SEND_F_VAL =         27;  // 2-byte
const int TP_SEND_G_VAL =         29;  // 2-byte
const int TP_SEND_H_VAL =         31;  // 2-byte
const int TP_SEND_I_VAL =         33;  // 2-byte
const int TP_SEND_J_VAL =         35;  // 2-byte
const int TP_SEND_MAX =           37;

const int TP_RCV_PACKET_TYPE =     0;  // 1-byte, type of remaining message
const int TP_RCV_X =               1;  // 1-byte x joystick
const int TP_RCV_Y =               2;  // 1-byte y joystick
const int TP_RCV_CMD =             3;  // 1-byte, "run" and "power" commands
const int TP_RCV_MODE =            4;  // 1-byte, mode command
const int TP_RCV_VALSET =          5;  // 1-byte valset 
const int TP_RCV_MSG_TYPE =        6;  // 1-byte message type
const int TP_RCV_MSG_VAL =         7;  // 2-byte message value
const int TP_RCV_MAX =             9;

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
const int TP_RCV_MSG_RESET =      16;  // Reset/Start/Stop something
const int TP_RCV_MSG_MODE =       17;  // Reset/Start/Stop something
const int TP_RCV_MSG_VALSET =     18;  // Reset/Start/Stop something

//const int TP_RCV_T_VALSET =        6;  // 2-byte valset value
//const int TP_RCV_U_VALSET =        8;  // 2-byte valset value
//const int TP_RCV_V_VALSET =       10;  // 2-byte valset value
//const int TP_RCV_W_VALSET =       12;  // 2-byte valset value
//const int TP_RCV_X_VALSET =       14;  // 2-byte valset value
//const int TP_RCV_Y_VALSET =       16;  // 2-byte valset value
//const int TP_RCV_Z_VALSET =       18;  // 2-byte valset value
//const int TP_RCV_MAX =            20;

// Parameterized commands;
const int CMD_BATTVOLT_VAL =       (CMD_PARAM +  1);
const int CMD_DEBUG              = (CMD_PARAM +  2);
const int CMD_A_VAL              = (CMD_PARAM +  3);
const int CMD_B_VAL              = (CMD_PARAM +  4);
const int CMD_C_VAL              = (CMD_PARAM +  5);
const int CMD_D_VAL              = (CMD_PARAM +  6);
const int CMD_E_VAL              = (CMD_PARAM +  7);
const int CMD_F_VAL              = (CMD_PARAM +  8);
const int CMD_G_VAL              = (CMD_PARAM +  9);
const int CMD_H_VAL              = (CMD_PARAM + 10);
const int CMD_I_VAL              = (CMD_PARAM + 11);
const int CMD_J_VAL              = (CMD_PARAM + 12);
const int CMD_K_VAL              = (CMD_PARAM + 13);
const int CMD_L_VAL              = (CMD_PARAM + 14);
const int CMD_GRAVITY            = (CMD_PARAM + 15);
const int CMD_RUN_STATE_STATUS   = (CMD_PARAM + 16);
const int CMD_MODE_STATUS        = (CMD_PARAM + 17);
const int CMD_HC_VAL             = (CMD_PARAM + 18);
const int CMD_VSET_STATUS        = (CMD_PARAM + 19);
const int CMD_DATA               = (CMD_PARAM + 20); 
const int CMD_END_DATA           = (CMD_PARAM + 21); 


// ------------------ Command constants received by TP -------------------
// Single byte commands, can only 
const int CMD_LED =                              0; // blink the led, two states
const int CMD_RESET =                            1;
const int CMD_HOME =                             2;
const int CMD_SEQ_LOAD =                         3;
const int CMD_SEQ_START =                        4;
const int CMD_PWR =                              5;
const int CMD_STREAM =                           6;
// limit 15

// Parameterized commands
const int CMD_xxxxxxxxx          = (CMD_PARAM +  0);
const int CMD_xxxxxxxxxxx        = (CMD_PARAM +  1);
const int CMD_T_SET              = (CMD_PARAM +  2);
const int CMD_U_SET              = (CMD_PARAM +  3);
const int CMD_V_SET              = (CMD_PARAM +  4);
const int CMD_W_SET              = (CMD_PARAM +  5);
const int CMD_X_SET              = (CMD_PARAM +  6);
const int CMD_Y_SET              = (CMD_PARAM +  7);
const int CMD_Z_SET              = (CMD_PARAM +  8); // Target Angle speed factor
const int CMD_LAMP               = (CMD_PARAM +  9); // For testing protocol.
const int CMD_Y                  = (CMD_PARAM + 10); // Joystick Y angle value
const int CMD_X                  = (CMD_PARAM + 11); // Joystick X angle value
const int CMD_MODE               = (CMD_PARAM + 12);
const int CMD_UP_VAL             = (CMD_PARAM + 13);
const int CMD_UP_END             = (CMD_PARAM + 14);
const int CMD_RUN_STATE          = (CMD_PARAM + 15);
const int CMD_ROTATE             = (CMD_PARAM + 16);
const int CMD_VAL_SET            = (CMD_PARAM + 17);
// limit 31

// Event interface



