int lastMWsFpsRight = 0;
int lastMWsFpsLeft = 0;
int rejectCountRight = 0;
int rejectCountLeft = 0;



/************************************************************************
 *  motorInitTp() 
 ************************************************************************/
void motorInitTp() {
  // Initialize the pwm frequency
  pwm_set_resolution(16);  
  pwm_setup(MOT_RIGHT_PWMH, TP_PWM_FREQUENCY, 1);  // on clock A
  pwm_setup(MOT_LEFT_PWMH, TP_PWM_FREQUENCY, 1);  // on clock A

  // Set the pin modes
  pinMode(MOT_RIGHT_MODE, OUTPUT);
  pinMode(MOT_LEFT_MODE, OUTPUT);
  pinMode(MOT_RIGHT_DIR, OUTPUT);
  pinMode(MOT_LEFT_DIR, OUTPUT);

  setMotor(MOTOR_RIGHT, BRAKE, 0);
  setMotor(MOTOR_LEFT, BRAKE, 0);

  setTargetSpeedRight(0.0);
  setTargetSpeedLeft(0.0);

  attachInterrupt(MOT_RIGHT_ENCA, encoderIsrRight, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeft, CHANGE);
//  Timer2.attachInterrupt(pollIsr);
//  Timer2.start(100);
}

//void pollIsr() {
//  tpDebug++;
//  tpDebug++;
//}

/*********************************************************
 *
 * encoderIsrRight()
 *
 *    Responds to interrupts from the encoder.
 *    Controls the motor directly from this isr.
 *
 *********************************************************/
void encoderIsrRight() {
  static boolean encAStat;
  int action = BRAKE;
  int pw = 0;
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsRight++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();  
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
    
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickPositionRight++;
  } 
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickPositionRight--;
  }
  wsMFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  wsMFpsRightSum += wsMFpsRight;
  wsMFpsRightCount++;
  
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;
  
  if (targetMFpsRight > 0L) {
    if (wsMFpsRight < targetMFpsRight) {              // < target speed
      action = FWD;
      pw = getAccelPw(wsMFpsRight, targetMFpsRight);
    }
    else if (wsMFpsRight < targetBrakeMFpsRight) {    // < brake speed
      action = COAST;
//      action = BRAKE;
    }
    else if (wsMFpsRight < targetRevMFpsRight) {      // < reverse speed
      action = BRAKE;
    }
    else {
      action = BKWD;
      pw = getDecelPw(wsMFpsRight, targetMFpsRight);
    }
  }
  else if (targetMFpsRight < 0L) {
    if (wsMFpsRight > targetMFpsRight) {
      action = BKWD;
      pw = getAccelPw(wsMFpsRight, targetMFpsRight);
    }
    else if (wsMFpsRight > targetBrakeMFpsRight) {
      action = COAST;
//      action = BRAKE;
    }
    else if (wsMFpsRight > targetRevMFpsRight) {
      action = BRAKE;
    }
    else {
      action = FWD;
      pw = getDecelPw(wsMFpsRight, targetMFpsRight);
    }
  }
  else { // STOP
    action = BRAKE;
  }
  setMotor(MOTOR_RIGHT, action, pw);
motorRightAction = action;
  
//addLog((long) (tickTimeRight),
//       (short) wsMFpsRight,
//       (short) targetMFpsRight,
//       (short) targetBrakeMFpsRight,
//       (short) targetRevMFpsRight, 
//       (short) pw,
//       (short) action
//       );
//
} // encoderIsrRight()


/************************************************************************
 *  encoderIsrLeft() 
 ************************************************************************/
void encoderIsrLeft() {
  static boolean encAStat;
  int action = BRAKE;
  int pw = 0;
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsLeft++;
    return;
  }
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  encAStat = encA;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  
  if (encA == encB) {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickPositionLeft--;
  } 
  else {
    tickPeriodLeft = (long)tickTimeLeft - (long) lastTickTime;
    tickPositionLeft++;
  }
  int wsMFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  wsMFpsLeftSum += wsMFpsLeft;
  wsMFpsLeftCount++;  
  
//runLog((long) tickTimeLeft, (long) mWsFpsLeft , (long) encA, (long) encB);
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;

  if (targetMFpsLeft > 0L) {
    if (wsMFpsLeft < targetMFpsLeft) {
      action = FWD;
      pw = getAccelPw(wsMFpsLeft, targetMFpsLeft);
    }
    else if (wsMFpsLeft < targetBrakeMFpsLeft) {
//      action = BRAKE;
      action = COAST;
    }
    else if (wsMFpsLeft < targetRevMFpsLeft) {
      action = BRAKE;
    }
    else {
      action = BKWD;
      pw = getDecelPw(wsMFpsLeft, targetMFpsLeft);
    }
  }
  else if (targetMFpsLeft < 0L) {
    if (wsMFpsLeft > targetMFpsLeft) {
      action = BKWD;
      pw = getAccelPw(wsMFpsLeft, targetMFpsLeft);
    }
    else if (wsMFpsLeft > targetBrakeMFpsLeft) {
//      action = BRAKE;
      action = COAST;
    }
    else if (wsMFpsLeft > targetRevMFpsLeft) {
      action = BRAKE;
    }
    else {
      action = FWD;
      pw = getDecelPw(wsMFpsLeft, targetMFpsLeft);
    }
  }
  else { // STOP
    action = BRAKE;
  }
  setMotor(MOTOR_LEFT, action, pw);
} // end encoderIsrLeft();



/************************************************************************
 *  getAccelPw() 
 ************************************************************************/
int getAccelPw(int wFps, int tFps) {
  int sum = abs(tFps) / 16;  // 0-1.0fps = 0-255
  sum = sum + (abs(tFps - wFps) / 16); // 0-1.0fps difference = 0-255
  if (sum > 255) sum = 255;
  return sum;
}
int getDecelPw(int wFps, int tFps) {
  int sum = -abs(tFps) / 8;  // 0-2.0fps = 0-255
  sum = sum + (abs(tFps - wFps) / 8); // 0-2.0fps difference = 0-255
  if (sum < 0) sum = 0;
  else if (sum > 255) sum = 255;
  return sum;
}



/*********************************************************
 *
 * checkMotorXXX()
 *
 *    Starts the motor if idle
 *
 *********************************************************/
void checkMotorRight() {
  int ws = (ENC_FACTOR_M / (micros() - tickTimeRight)); // speed in milli-fps
//runLog((long) ws, (long) targetMFpsRight , 9, 99);
  if (ws < 100) { // less than ~0.1 fps?
    if (ws < abs(targetMFpsRight / 2)) { // less than 1/2 target speed?
      if (targetMFpsRight > 0) {
        setMotor(MOTOR_RIGHT, FWD, 70);
      }
      else {
        setMotor(MOTOR_RIGHT, BKWD, 70);
      }
    }
  }
}


/************************************************************************
 *  checkMotorLeft() 
 ************************************************************************/
void checkMotorLeft() {
  int ws = (ENC_FACTOR_M / (micros() - tickTimeLeft)); // speed in milli-fps
  if (ws < 100) { // less than ~0.1 fps?
    if (ws < abs(targetMFpsLeft / 2)) { // less than 1/2 target speed?
      if (targetMFpsLeft > 0) {
        setMotor(MOTOR_LEFT, FWD, 70);
      }
      else {
        setMotor(MOTOR_LEFT, BKWD, 70);
      }
    }
  }
}



/*********************************************************
 *
 * setTargetSpeedRight()
 *
 *    Set the targetTickPeriodRight targetBrakePeriodRight
 *    and the waitPeriodRight given the targetSpeed.
 *
 *********************************************************/
void setTargetSpeedRight(double targetSpeed) {
  targetSpeedRight = targetSpeed;
  targetMFpsRight = (int) (targetSpeed * 1000.0);//////////////////////////////
  
  if (targetMFpsRight > 0.02f) {
    int brakePctRight = (int) (targetSpeed * 1020.0);// 2% /////////////////////////
    int revPctRight = (int) (targetSpeed * 1050.0);// 5% ///////////////////////////
    int brakeSpdRight = (int) targetSpeed + 0.05;
    int revSpdRight = (int) targetSpeed + 0.10;
    if (brakePctRight > brakeSpdRight) {
      targetBrakeMFpsRight = brakePctRight;
      targetRevMFpsRight = revPctRight;
    }
    else {
      targetBrakeMFpsRight = brakeSpdRight;
      targetRevMFpsRight = revSpdRight;
    }
  }
  else if (targetMFpsRight < -0.02f) {
    int brakePctRight = (int) (targetSpeed * 1020.0);// 2% /////////////////////////
    int revPctRight = (int) (targetSpeed * 1050.0);// 5% ///////////////////////////
    int brakeSpdRight = (int) targetSpeed - 0.05;
    int revSpdRight = (int) targetSpeed - 0.10;
    if (brakePctRight < brakeSpdRight) {
      targetBrakeMFpsRight = brakePctRight;
      targetRevMFpsRight = revPctRight;
    }
    else {
      targetBrakeMFpsRight = brakeSpdRight;
      targetRevMFpsRight = revSpdRight;
    }
  }
  else {
    targetMFpsRight = 0L;
  }
}


/********************* setTargetSpeedLeft() ********************/
void setTargetSpeedLeft(double targetSpeed) {
  targetSpeedLeft = targetSpeed;
  targetMFpsLeft = (int) (targetSpeed * 1000.0);//////////////////////////////
  
  if (targetMFpsLeft > 0.02f) {
//    targetDirectionLeft = FWD;
    int brakePctLeft = (int) (targetSpeed * 1020.0);// 2% /////////////////////////
    int revPctLeft = (int) (targetSpeed * 1050.0);// 5% ///////////////////////////
    int brakeSpdLeft = (int) targetSpeed + 0.05;
    int revSpdLeft = (int) targetSpeed + 0.10;
    if (brakePctLeft > brakeSpdLeft) {
      targetBrakeMFpsLeft = brakePctLeft;
      targetRevMFpsLeft = revPctLeft;
    }
    else {
      targetBrakeMFpsLeft = brakeSpdLeft;
      targetRevMFpsLeft = revSpdLeft;
    }
  }
  else if (targetMFpsLeft < -0.02f) {
//    targetDirectionLeft = BKWD;
    int brakePctLeft = (int) (targetSpeed * 1020.0);// 2% /////////////////////////
    int revPctLeft = (int) (targetSpeed * 1050.0);// 5% ///////////////////////////
    int brakeSpdLeft = (int) targetSpeed - 0.05;
    int revSpdLeft = (int) targetSpeed - 0.10;
    if (brakePctLeft < brakeSpdLeft) {
      targetBrakeMFpsLeft = brakePctLeft;
      targetRevMFpsLeft = revPctLeft;
    }
    else {
      targetBrakeMFpsLeft = brakeSpdLeft;
      targetRevMFpsLeft = revSpdLeft;
    }
  }
  else {
//    targetDirectionLeft = STOP;
    targetMFpsLeft = 0L;
  }
}

/*********************************************************
 * setMotorMode()
 *********************************************************/
// void setMotorMode(int mm) {
//   motorMode = mm;
//   if (mm == MM_DRIVE_BRAKE) {
//     g_APinDescription[MOT_RIGHT_MODE].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_MODE].ulPin; // HIGHT
//     g_APinDescription[MOT_LEFT_MODE].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_MODE].ulPin; // HIGHT
//   }
//   else {
//     g_APinDescription[MOT_RIGHT_MODE].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_MODE].ulPin;   // LOW
//     g_APinDescription[MOT_LEFT_MODE].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_MODE].ulPin;   // LOW
//   }
//
// }
 
 
 
/*********************************************************
 * setMotor()
 *********************************************************/
void setMotor(int motor, int action, int pw) {
  int pinMm;
  int pinDir;
  int pinPwmH;
  

  if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  
  pw = pw * 255; // Convert for new pwm control
  
  if (motor == MOTOR_RIGHT) {
    pinMm = MOT_RIGHT_MODE;
    pinDir = MOT_RIGHT_DIR;
    pinPwmH = MOT_RIGHT_PWMH;
    actionRight = action;
  }
  else {
    pinMm = MOT_LEFT_MODE ;
    pinDir = MOT_LEFT_DIR;
    pinPwmH = MOT_LEFT_PWMH;
    actionLeft = action;
  }
  if (mode == MODE_TP6) {
    if (!isRunning) action = BRAKE;
  }
  if ((action == COAST) || (action == BRAKE)) {
    pw = 0;
  }
  
  pwm_write_duty(pinPwmH, pw);  // 50% duty cycle on Pin 6
  if ((action == FWD) || (action == BKWD)) { // Set the mode pin.
    if (motorMode == MM_DRIVE_BRAKE) {
      g_APinDescription[pinMm].pPort -> PIO_SODR = g_APinDescription[pinMm].ulPin; // HIGHT
    }
    else { // DRIVE_COAST
      g_APinDescription[pinMm].pPort -> PIO_CODR = g_APinDescription[pinMm].ulPin;   // LOW
    }
    if (action == FWD) {
      g_APinDescription[pinDir].pPort -> PIO_SODR = g_APinDescription[pinDir].ulPin; // HIGH
    }
    else {
      g_APinDescription[pinDir].pPort -> PIO_CODR = g_APinDescription[pinDir].ulPin;   // LOW
    }
  }
  else if (action == COAST) {
    g_APinDescription[pinMm].pPort -> PIO_CODR = g_APinDescription[pinMm].ulPin; // LOW
  }
  else { // BRAKE
    g_APinDescription[pinMm].pPort -> PIO_SODR = g_APinDescription[pinMm].ulPin; // HIGH
  }
}
//// example: digitalWriteDirect(10, HIGH)
//inline void digitalWriteDirect(int pin, boolean val){
//  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
//  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
//}




/*********************************************************
 *
 *  getSpeedXXX() average speed since last read
 *
 *********************************************************/
void readSpeedRight() {
  noInterrupts();
  long sum = wsMFpsRightSum;
  int count =  wsMFpsRightCount;
  wsMFpsRightSum = 0L;
  wsMFpsRightCount = 0;
  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeRight);
    if (newMFps < mFpsRight) mFpsRight = newMFps; // Set new if lower
  }
  else {
    mFpsRight = sum / count;
  }
  fpsRight =((double) mFpsRight) / 1000.0;
  fpsRightBuf[++fpsRightBufPtr % FPS_BUF_SIZE] = fpsRight;
}

void readSpeedLeft() {
  noInterrupts();
  long sum = wsMFpsLeftSum;
  int count =  wsMFpsLeftCount;
  wsMFpsLeftSum = 0L;
  wsMFpsLeftCount = 0;
  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeLeft);
    if (newMFps < mFpsLeft) mFpsLeft = newMFps; // Set new if lower
  }
  else {
    mFpsLeft = sum / count;
  }
  fpsLeft =((double) mFpsLeft) / 1000.0;
  fpsLeftBuf[++fpsLeftBufPtr % FPS_BUF_SIZE] = fpsLeft;
}



/*********************************************************
 *
 * readSpeed()
 *
 *    Sets the speed variables for both wheels and 
 *    sets average.
 *
 *********************************************************/
void readSpeed() {
  readSpeedRight();
  readSpeedLeft();
  tickPosition = tickPositionRight + tickPositionLeft;
  wheelSpeedFps = (fpsLeft + fpsRight)/ 2.0D;
  mWheelSpeedFps = (mFpsRight + mFpsLeft) / 2.0D;
}



