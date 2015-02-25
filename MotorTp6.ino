


/**************************************************************************.
 *  motorInitTp6() 
 **************************************************************************/
void motorInitTp6() {
  // Set the pin modes
  pinMode(MOT_RIGHT_PWML, OUTPUT);
  pinMode(MOT_RIGHT_DIR, OUTPUT);
  pinMode(MOT_LEFT_PWML, OUTPUT);
  pinMode(MOT_LEFT_DIR, OUTPUT);

  setMotor(MOTOR_RIGHT, COAST, 0);
  setMotor(MOTOR_LEFT, COAST, 0);

  setTargetSpeed6Right(0.0);
  setTargetSpeed6Left(0.0);

  attachInterrupt(MOT_RIGHT_ENCA, encoderIsr6Right, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsr6Left, CHANGE);
}


/**************************************************************************.
 *
 * encoderIsr6Right()
 *
 *    Responds to interrupts from the encoder.
 *    Controls the motor directly from this isr.
 *
 ***************************************************************************/
void encoderIsr6Right() {
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
  mWsFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  mWsFpsRightSum += mWsFpsRight;
  mWsFpsRightCount++;
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
addLog((long) tickTimeRight,
       (short) tickPositionRight,
       (short) action,
       (short) pw,
       (short) mWsFpsRight, 
       (short) (tp5FpsRight * 100.0),
       (short) (tp5FpsLeft * 100.0));
} // encoderIsr6Right()


/**************************************************************************.
 *  encoderIsrLeft() 
 **************************************************************************/
void encoderIsr6Left() {
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
  int mWsFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  mWsFpsLeftSum += mWsFpsLeft;
  mWsFpsLeftCount++;  
  
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
} // end encoderIsr6Left();



/**************************************************************************.
 *  getAccelPw() 
 **************************************************************************/
int getAccelPw6(int wFps, int tFps) {
  int sum = abs(tFps) / 8;  // 0-1.0fps = 0-255
  sum = sum + (abs(tFps - wFps) / 8); // 0-1.0fps difference = 0-255
  if (sum > 255) sum = 255;
  return sum;
}
int getDecelPw6(int wFps, int tFps) {
  int sum = -abs(tFps) / 8;  // 0-2.0fps = 0-255
  sum = sum + (abs(tFps - wFps) / 8); // 0-2.0fps difference = 0-255
  if (sum < 0) sum = 0;
  else if (sum > 255) sum = 255;
  return sum;
}



/**************************************************************************.
 *
 * checkMotorXXX()
 *
 *    Starts the motor if idle
 *
 **************************************************************************/
void checkMotor6Right() {
  int ws = (ENC_FACTOR_M / (micros() - tickTimeRight)); // speed in milli-fps
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


/**************************************************************************.
 *  checkMotorLeft() 
 **************************************************************************/
void checkMotor6Left() {
  int ws = (ENC_FACTOR_M / (micros() - tickTimeLeft)); // speed in milli-fps
  if (ws < 100) { // less than ~0.1 fps?
    if (ws < abs(targetMFpsLeft / 2)) { // less than 1/2 target speed?
      if (targetMFpsLeft > 0) {
        setMotor(MOTOR_LEFT, FWD, 199);
      }
      else {
        setMotor(MOTOR_LEFT, BKWD, 199);
      }
    }
  }
}



/**************************************************************************.
 *
 * setTargetSpeedRight()
 *
 *    Set the targetTickPeriodRight targetBrakePeriodRight
 *    and the waitPeriodRight given the targetSpeed.
 *
 **************************************************************************/
void setTargetSpeed6Right(float targetSpeed) {
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
void setTargetSpeed6Left(float targetSpeed) {
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

//// example: digitalWriteDirect(10, HIGH)
//inline void digitalWriteDirect(int pin, boolean val){
//  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
//  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
//}




/**************************************************************************.
 *
 *  getSpeedXXX() average speed since last read
 *
 **************************************************************************/
void readSpeed6Right() {
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

void readSpeed6Left() {
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



/**************************************************************************.
 *
 * readSpeed()
 *
 *    Sets the speed variables for both wheels and 
 *    sets average.
 *
 **************************************************************************/
void readSpeed6() {
  readSpeed6Right();
  readSpeed6Left();
  tickPosition = tickPositionRight + tickPositionLeft;
  wheelSpeedFps = (fpsLeft + fpsRight)/2.0f;
  mWheelSpeedFps = (mAverageFpsRight + mAverageFpsLeft) /2;
}



