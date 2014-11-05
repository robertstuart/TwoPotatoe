int lastMWsFpsRight = 0;
int lastMWsFpsLeft = 0;
int rejectCountRight = 0;
int rejectCountLeft = 0;



/************************************************************************
 *  motorInitTp() 
 ************************************************************************/
void motorInitTp() {
  // Set the pin modes
  pinMode(MOT_RIGHT_PWML, OUTPUT);
  pinMode(MOT_RIGHT_DIR, OUTPUT);
  pinMode(MOT_LEFT_PWML, OUTPUT);
  pinMode(MOT_LEFT_DIR, OUTPUT);

  setMotor(MOTOR_RIGHT, COAST, 0);
  setMotor(MOTOR_LEFT, COAST, 0);

  setTargetSpeedRight(0.0);
  setTargetSpeedLeft(0.0);

  attachInterrupt(MOT_RIGHT_ENCA, encoderIsrRight, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeft, CHANGE);
}


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
    interruptErrors++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();  
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
    
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickDistanceRight++;
  } 
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickDistanceRight--;
  }
  mWsFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  mWsFpsRightSum += mWsFpsRight;
  mWsFpsRightCount++;
//runLog((long) tickTimeRight, (long) mWsFpsRight , (long) encA, (long) encB);
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;
  
  if (targetMFpsRight > 0L) {
    if (mWsFpsRight < targetMFpsRight) {
      action = FWD;
      pw = getAccelPw(mWsFpsRight, targetMFpsRight);
    }
    else if (mWsFpsRight < targetBrakeMFpsRight) {
//      action = COAST;
      action = BRAKE;
    }
    else if (mWsFpsRight < targetRevMFpsRight) {
      action = BRAKE;
    }
    else {
      action = BKWD;
      pw = getDecelPw(mWsFpsRight, targetMFpsRight);
    }
  }
  else if (targetMFpsRight < 0L) {
    if (mWsFpsRight > targetMFpsRight) {
      action = BKWD;
      pw = getAccelPw(mWsFpsRight, targetMFpsRight);
    }
    else if (mWsFpsRight > targetBrakeMFpsRight) {
//      action = COAST;
      action = BRAKE;
    }
    else if (mWsFpsRight > targetRevMFpsRight) {
      action = BRAKE;
    }
    else {
      action = FWD;
      pw = getDecelPw(mWsFpsRight, targetMFpsRight);
    }
  }
  else { // STOP
    action = BRAKE;
  }
  setMotor(MOTOR_RIGHT, action, pw);
//tp7Log(targetMFpsRight, mWsFpsRight, action, pw);
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
    interruptErrors++;
    return;
  }
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  encAStat = encA;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  
  if (encA == encB) {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickDistanceLeft--;
  } 
  else {
    tickPeriodLeft = (long)tickTimeLeft - (long) lastTickTime;
    tickDistanceLeft++;
  }
  int mWsFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  mWsFpsLeftSum += mWsFpsLeft;
  mWsFpsLeftCount++;  
  
//runLog((long) tickTimeLeft, (long) mWsFpsLeft , (long) encA, (long) encB);
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;

  if (targetMFpsLeft > 0L) {
    if (mWsFpsLeft < targetMFpsLeft) {
      action = FWD;
      pw = getAccelPw(mWsFpsLeft, targetMFpsLeft);
    }
    else if (mWsFpsLeft < targetBrakeMFpsLeft) {
      action = BRAKE;
//      action = COAST;
    }
    else if (mWsFpsLeft < targetRevMFpsLeft) {
      action = BRAKE;
    }
    else {
      action = BKWD;
      pw = getDecelPw(mWsFpsLeft, targetMFpsLeft);
    }
  }
  else if (targetMFpsLeft < 0L) {
    if (mWsFpsLeft > targetMFpsLeft) {
      action = BKWD;
      pw = getAccelPw(mWsFpsLeft, targetMFpsLeft);
    }
    else if (mWsFpsLeft > targetBrakeMFpsLeft) {
      action = BRAKE;
//      action = COAST;
    }
    else if (mWsFpsLeft > targetRevMFpsLeft) {
      action = BRAKE;
    }
    else {
      action = FWD;
      pw = getDecelPw(mWsFpsLeft, targetMFpsLeft);
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
  int sum = abs(tFps) / 8;  // 0-1.0fps = 0-255
  sum = sum + (abs(tFps - wFps) / 8); // 0-1.0fps difference = 0-255
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
        setMotor(MOTOR_RIGHT, FWD, 199);
      }
      else {
        setMotor(MOTOR_RIGHT, BKWD, 199);
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
        setMotor(MOTOR_LEFT, FWD, 60);
      }
      else {
        setMotor(MOTOR_LEFT, BKWD, 60);
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
void setTargetSpeedRight(float targetSpeed) {
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
void setTargetSpeedLeft(float targetSpeed) {
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
 *
 * setMotor()
 *
 *    Set the motor to the specified state.
 *
 *********************************************************/
void setMotor(int motor, int state, int pw) {
  int pinPwmL;
  int pinDir;
  int pinPwmH;

  if (motor == MOTOR_RIGHT) {
    pinPwmL = MOT_RIGHT_PWML;
    pinDir = MOT_RIGHT_DIR;
    pinPwmH = MOT_RIGHT_PWMH;
    if (state == FWD) state = BKWD;
    else if (state == BKWD) state = FWD;
    actionRight = state;
//runLog(timeMicroseconds, state, pw, 0);
  }
  else {
    pinPwmL = MOT_LEFT_PWML; // Reversed
    pinDir = MOT_LEFT_DIR;
    pinPwmH = MOT_LEFT_PWMH;
    actionLeft = state;
  }
  if ((mode == MODE_TP5) || (mode == MODE_TP6)) {
    if ((tpState & TP_STATE_RUNNING) == 0) state = COAST;
  }
  
  switch(state) {
  case FWD:
    g_APinDescription[pinPwmL].pPort -> PIO_SODR = g_APinDescription[pinPwmL].ulPin; // HIGHT
    g_APinDescription[pinDir].pPort -> PIO_CODR = g_APinDescription[pinDir].ulPin;   // LOW
    analogWrite(pinPwmH, pw);
    break;
  case BKWD:
    g_APinDescription[pinPwmL].pPort -> PIO_SODR = g_APinDescription[pinPwmL].ulPin; // HIGH
    g_APinDescription[pinDir].pPort -> PIO_SODR = g_APinDescription[pinDir].ulPin; // HIGH
    analogWrite(pinPwmH, pw);
    break;
  case COAST:
    g_APinDescription[pinPwmL].pPort -> PIO_CODR = g_APinDescription[pinPwmL].ulPin; // LOW
    analogWrite(pinPwmH, 0);
    break;
  case BRAKE:
    g_APinDescription[pinPwmL].pPort -> PIO_SODR = g_APinDescription[pinPwmL].ulPin; // HIGH
    analogWrite(pinPwmH, 0);
    break;
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
  long sum = mWsFpsRightSum;
  int count =  mWsFpsRightCount;
  mWsFpsRightSum = 0L;
  mWsFpsRightCount = 0;
  interrupts();
  if (count == 0) mAverageFpsRight = 0;
  else mAverageFpsRight = sum / count;
  fpsRight =((float) mAverageFpsRight) / 1000.0;
}

void readSpeedLeft() {
  noInterrupts();
  long sum = mWsFpsLeftSum;
  int count =  mWsFpsLeftCount;
  mWsFpsLeftSum = 0L;
  mWsFpsLeftCount = 0;
  interrupts();
  if (count == 0) mAverageFpsLeft = 0;
  else mAverageFpsLeft = sum / count;
  fpsLeft =((float) mAverageFpsLeft) / 1000.0;
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
  tickDistance = tickDistanceRight + tickDistanceLeft;
  wheelSpeedFps = (fpsLeft + fpsRight)/2.0f;
  mWheelSpeedFps = (mAverageFpsRight + mAverageFpsLeft) /2;
}



