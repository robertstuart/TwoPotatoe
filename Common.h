// Modes of operation
const int MODE_PID1           = 0;  // Original pid on board
const int MODE_MOTOR_PW       = 1;  // pw values sent from controller
const int MODE_DRIVE          = 2;  // sequences batch-loaded from controller
const int MODE_TP4            = 3;  // Motor speed controlled from interrupts
const int MODE_TP_SPEED       = 4;  // on-board motor speed
const int MODE_TP_SEQUENCE    = 5;  // sequences batch-loaded from controller
const int MODE_TP3            = 6;  // 
const int MODE_HOME           = 7;

// States for tp
const int STATE_RESTING         = 0;  // motors off, not trying to run
const int STATE_READY           = 1;  // will move to RUN state when upright and gravity
const int STATE_RUNNING         = 2;  // running, will move to READY if not upright or no gravity

// value sets
const int VAL_SET_A           = 0;
const int VAL_SET_B           = 1;
const int VAL_SET_C           = 2;

// substates of the run state
const int RUNSTATE_NORMAL       = 0;
const int RUNSTATE_DOCKING      = 1;
const int RUNSTATE_DOCKED       = 2;  // needed?
const int RUNSTATE_SEEKING_HOME = 3;
const int RUNSTATE_HOME         = 4;



// For commands that have two states, (0x20) bit indicates the state
// Single byte commands are < 32
const int CMD_SINGLE = 0x80;
const int CMD_PARAM =   (CMD_SINGLE | 0x40);  // Start for parameterized commands
const int CMD_SINGLE_FLAG = 0x20;  // Indicate true or false for single-byte commands


// ----------------- Command constants sent from TP --------------------
// Single-byte commands, 
const int CMD_PACKET_END         = (CMD_SINGLE + 1);
const int CMD_SEQUENCE_END       = (CMD_SINGLE + 2);
const int CMD_UPLOAD_START       = (CMD_SINGLE + 3);

// Parameterized commands;
const int CMD_BATTVOLT_VAL       = (CMD_PARAM +  1);
const int CMD_DEBUG_MSG          = (CMD_PARAM +  2);
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
const int CMD_RUN_STATE_STAT     = (CMD_PARAM + 16); // status
const int CMD_RUN_SUBSTATE_STAT  = (CMD_PARAM + 17); // status substate of run state
const int CMD_X_STAT             = (CMD_PARAM + 18);  
const int CMD_Y_STAT             = (CMD_PARAM + 19);
const int CMD_MODE_STAT          = (CMD_PARAM + 20);
const int CMD_HC_VAL             = (CMD_PARAM + 21);
const int CMD_VAL_SET_STAT       = (CMD_PARAM + 22);
const int CMD_XXXX               = (CMD_PARAM + 23);
const int CMD_PING_REPORT        = (CMD_PARAM + 24);
const int CMD_MSG                = (CMD_PARAM + 25); 
const int CMD_FROM_TP_PING       = (CMD_PARAM + 26); 
const int CMD_HC_TP_ERRORS       = (CMD_PARAM + 27);  // from TP
const int CMD_TP_HC_ERRORS       = (CMD_PARAM + 28);  // from HC


// ------------------ Command constants received by TP -------------------
// Single byte commands, can only 
const int CMD_LED                = (CMD_SINGLE +  1); // blink the led, two states
const int CMD_A_DBG              = (CMD_SINGLE +  2);
const int CMD_B_DBG              = (CMD_SINGLE +  3);
const int CMD_C_DBG              = (CMD_SINGLE +  4);
const int CMD_D_DBG              = (CMD_SINGLE +  5);
const int CMD_E_DBG              = (CMD_SINGLE +  6);
const int CMD_F_DBG              = (CMD_SINGLE +  7);
const int CMD_G_DBG              = (CMD_SINGLE +  8);
const int CMD_H_DBG              = (CMD_SINGLE +  9);
const int CMD_I_DBG              = (CMD_SINGLE + 10);
const int CMD_J_DBG              = (CMD_SINGLE + 11);
const int CMD_RESET              = (CMD_SINGLE + 12);
const int CMD_HOME               = (CMD_SINGLE + 13);
const int CMD_ALIVE              = (CMD_SINGLE + 14);
const int CMD_SEQ_LOAD           = (CMD_SINGLE + 15);
const int CMD_SEQ_START          = (CMD_SINGLE + 16);
const int CMD_PWR                = (CMD_SINGLE + 17);

// Parameterized commands
const int CMD_T_SET              = (CMD_PARAM +  1);
const int CMD_U_SET              = (CMD_PARAM +  2);
const int CMD_V_SET              = (CMD_PARAM +  3);
const int CMD_W_SET              = (CMD_PARAM +  4);
const int CMD_X_SET              = (CMD_PARAM +  5);
const int CMD_Y_SET              = (CMD_PARAM +  6);
const int CMD_Z_SET              = (CMD_PARAM +  7); // Target Angle speed factor
const int CMD_XXXXXXXXXXXXXXX    = (CMD_PARAM +  8); // Accel offset
const int CMD_Y                  = (CMD_PARAM +  9); // Joystick Y angle value
const int CMD_X                  = (CMD_PARAM + 10); // Joystick X angle value
const int CMD_GENERAL            = (CMD_PARAM + 11);
const int CMD_MODE               = (CMD_PARAM + 12);
const int CMD_UP_VAL             = (CMD_PARAM + 13);
const int CMD_UP_END             = (CMD_PARAM + 14);
const int CMD_RUN_STATE          = (CMD_PARAM + 15);
const int CMD_RUN_SUBSTATE       = (CMD_PARAM + 16);
const int CMD_ROTATE             = (CMD_PARAM + 17);
const int CMD_VAL_SET            = (CMD_PARAM + 18);
const int CMD_TO_TP_PING         = (CMD_PARAM + 19);  // 1U    xent from HC & PC      



