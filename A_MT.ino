//  Algorithm for testing motor response
/******************************************************
 *
 *    Algorithm for testing motor response
 *
 *    Read the durArray[] and motArray[] to set the motor 
 *    state to BRAKE, COAST, FWD, STOP, or BKWD.
 *    durArray[] stores durations in microseconds
 *
 ******************************************************/

#define MOTOR_INTS 500
#define MOTOR_SEQS 10

#define DATAFLAG_ENCODE_FWD 0L;
#define DATAFLAG_ENCODE_BKWD 1L;
#define DATAFLAG_MOTOR_FWD 2L;
#define DATAFLAG_MOTOR_BKWD 3L;
#define DATAFLAG_MOTOR_COAST 4L;
#define DATAFLAG_MOTOR_BRAKE 5L;
#define DATAFLAG_GYROX 6L;

unsigned long mtIntArray[MOTOR_INTS];  // Stores the times of the interrupts
unsigned int mtIntIndex = 0;  // pointer to above array
unsigned long mtTimes[MOTOR_SEQS];
unsigned long mtDurArray[MOTOR_SEQS];
int mtMotorStateArray[MOTOR_SEQS];
unsigned int mtSeqIndex = 0;
unsigned long  mtTimeTrigger = 0L;
int gyroMtXRaw = 0;

/******************************************************
 *
 *    Initialize the motor 
 *
 *
 *
 ******************************************************/
void mtInit() {
  
  // Set up IMU
//  compass.init(LSM303DLHC_DEVICE, 0);
//  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
//  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
//  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
//  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth
//  gyro.writeReg(L3G_CTRL_REG2, 0x09); // Highpass 0.09

  // TODO revisit these parameters
//***test  gyro.writeReg(L3G_CTRL_REG2, 0x00); // 250 dps full scale
//***test  gyro.writeReg(L3G_CTRL_REG5, 0x10); // high-pass enable
//***test  gyro.writeReg(L3G_CTRL_REG2, 0x03); // high-pass frequency
//***test  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale

  accelgyro.initialize();
//  accelgyro.setDLPFMode(6);
  delay(100);
//  debugStr((char*) (accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"), " ");

  attachInterrupt(MOT_RIGHT_ENCA, mtIsrRight, CHANGE);
  detachInterrupt(MOT_RIGHT_ENCA);
  setMotor(MOTOR_RIGHT, COAST);
  mtIntIndex = 0;
  mtSeqIndex = 0;
  mtRun = true;
  mtTimeTrigger = 0L;
}


/******************************************************
 *
 *    Interrupt routines 
 *
 *
 *
 ******************************************************/
void mtIsrRight() {
  // Put in the gyro value
  mtIntArray[mtIntIndex] = (gyroMtXRaw & 0xFFFFFFF8) | DATAFLAG_GYROX;
  if (mtIntIndex < MOTOR_INTS) {
    mtIntIndex++;
  }
  
  // put in the encoder interrupt
  unsigned long t = micros() & 0xFFFFFFF8;  // mask off lower 3 bits
  if (digitalRead(MOT_RIGHT_ENCA) == digitalRead(MOT_RIGHT_ENCB)) {
    mtIntArray[mtIntIndex] = t | DATAFLAG_ENCODE_FWD;
  } 
  else {
    mtIntArray[mtIntIndex] = t | DATAFLAG_ENCODE_BKWD;
  }
  if (mtIntIndex < MOTOR_INTS) {
    mtIntIndex++;
  }
}

/******************************************************
 *
 *    a_mt() 
 *
 *
 *
 ******************************************************/
void a_mt() {
//  // Read the gyro
////  gyro.readX();   
////  gyroMtXRaw = gyro.g.x;  // 
//    gyroMtXRaw = accelgyro.getRotationX();
//
//  // check time & go to next sequence  
//  int motorCmd = MOTOR_BRAKE;
//  int motorData = DATAFLAG_MOTOR_BRAKE;
//  unsigned long t = micros();
//  if (t > mtTimeTrigger) {
//    unsigned long dur = mtDurArray[mtSeqIndex];
//    int motorState = mtMotorStateArray[mtSeqIndex];
//    if ((mtSeqIndex >= MOTOR_SEQS) || (motorState == END_MARKER)) {
//      detachInterrupt(0);
//      MOTOR_PORT_RIGHT = MOTOR_COAST;
//      mtRun = false;
//      mtDump();
//      return;
//    }
//
//    switch (motorState) { // get const for motor and data
//    case FWD:
//      motorData = DATAFLAG_MOTOR_FWD;
//      motorCmd = MOTOR_FWD;
//      break;
//    case BKWD:
//      motorData = DATAFLAG_MOTOR_BKWD;
//      motorCmd = MOTOR_BKWD;
//      break;
//    case BRAKE:
//      motorData = DATAFLAG_MOTOR_BRAKE;
//      motorCmd = MOTOR_BRAKE;
//      break;
//    case COAST:
//    case STOP:
//      motorData = DATAFLAG_MOTOR_COAST;
//      motorCmd = MOTOR_COAST;
//      break;
//    default:
//      motorData = DATAFLAG_MOTOR_FWD;
//      debugInt("Bad motorState: ", motorCmd);
//      break;
//    }
//
//    // Put in array if room.
//    if (mtIntIndex < MOTOR_INTS) { 
//      noInterrupts();
//      mtIntArray[mtIntIndex++] = (t & 0xFFFFFFF8L) | motorData;   // Mark the mtIntArray
//      interrupts();
//    }
//    mtTimeTrigger = t + dur;
//    MOTOR_PORT_RIGHT = motorCmd;  
//    mtSeqIndex++;
//  }
}

/******************************************************
 *
 *    mtDump()
 *
 *    Dump the interrupt data to the PC
 *
 ******************************************************/
void mtDump() {
  for (int i = 0; i < mtIntIndex; i++) {
    send5(CMD_DATA, mtIntArray[i]);
    delay(20);
  }
  send2(CMD_END_DATA, mtIntIndex);
}


