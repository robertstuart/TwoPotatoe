/************************************************************************************************************



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

  digitalWrite(MOT_RIGHT_MODE, HIGH);  // Drive-brake mode.
  digitalWrite(MOT_LEFT_MODE, HIGH);  // Drive-brake mode.

  //  setMotor(MOTOR_RIGHT, BRAKE, 0);
  //  setMotor(MOTOR_LEFT, BRAKE, 0);
  setMotor(MOTOR_RIGHT, COAST, 0);
  setMotor(MOTOR_LEFT, COAST, 0);

  setTargetSpeedRight(0.0);
  setTargetSpeedLeft(0.0);

  attachInterrupt(MOT_RIGHT_ENCA, encoderIsrRight, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeft, CHANGE);
  //  Timer2.attachInterrupt(pollIsr);
  //  Timer2.start(100);
}

void encoderIsrRight() {
  static int lastTickPeriodRight = 0;
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

  boolean encZ = (!!(g_APinDescription[MOT_RIGHT_ENCZ].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCZ].ulPin)) ? true : false;
  tArray[tickArrayPtr] = tickPeriodRight;
  uArray[tickArrayPtr] = encZ;
  tickArrayPtr = ++tickArrayPtr % TICK_ARRAY_SIZE;

  wsMFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps

  // Comment out this block to not average fps over two ticks.
  //  if (wsMFpsRight > 500 ) {
  //    wsMFpsRight = ENC_FACTOR_M / ((lastTickPeriodRight + tickPeriodRight) / 2);
  //    lastTickPeriodRight = tickPeriodRight;
  //  }

  wsMFpsRightSum += wsMFpsRight;
  wsMFpsRightCount++;

  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;
  if (isDiagnostics) return;

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
} // encoderIsrRight()


void encoderIsrLeft() {
  static int lastTickPeriodLeft = 0;
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

  // Comment out this block to not average fps over two ticks.
  //  if (wsMFpsLeft > 500 ) {
  //    wsMFpsLeft = ENC_FACTOR_M / ((lastTickPeriodLeft + tickPeriodLeft) / 2);
  //    lastTickPeriodLeft = tickPeriodLeft;
  //  }

  wsMFpsLeftSum += wsMFpsLeft;
  wsMFpsLeftCount++;

  //runLog((long) tickTimeLeft, (long) mWsFpsLeft , (long) encA, (long) encB);
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;
  if (isDiagnostics) return;

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



const float CHECK_MULT = 5.0;
void checkMotorRight() {
  int dir;
  float fpsAbs;
  
  if (targetSpeedRight < 0.0) {
    fpsAbs = -targetSpeedRight;
    dir = BKWD;
  } else {
    fpsAbs = targetSpeedRight;
    dir = FWD;
  }
  float ticMs = ((float) (micros() - tickTimeRight)) / 1000.0;
  int pw  = (int) (fpsAbs * ticMs * CHECK_MULT);
  pw += 8 + ((int) (fpsAbs * 11.1));
  setMotor(MOTOR_RIGHT, dir, pw);
}

void checkMotorLeft() {
  int dir;
  float fpsAbs;
  
  if (targetSpeedLeft < 0.0) {
    fpsAbs = -targetSpeedLeft;
    dir = BKWD;
  } else {
    fpsAbs = targetSpeedLeft;
    dir = FWD;
  }
  float ticMs = ((float) (micros() - tickTimeLeft)) / 1000.0;
  int pw  = (int) (fpsAbs * ticMs * CHECK_MULT);
  pw += 8 + ((int) (fpsAbs * 11.1));
  setMotor(MOTOR_LEFT, dir, pw);
}


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
  } else {
    targetMFpsRight = 0L;
  }
}


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



const float MOTOR_GAIN_RW = 5.0;
static const int ZERO_PW = 32766;  // Center of pulse width range. FPS = 0;
static const int DEAD_ZONE = 600; // Zone around ZERO_PW that gives zero FPS, 775
static const float PW_VS_FPS = 1100.0; // change in PW gives change of 1.0 FPS, 775 12V motor

void setRightTargetSpeed(float targetFps) {
  int dead = (targetFps < 0) ? - DEAD_ZONE : DEAD_ZONE;
  int target = (targetFps * PW_VS_FPS) + dead;
  targetPwRight = ZERO_PW + target;
  targetMFpsRight = (int) (targetFps * 1000.0); 
}
void setRightMotorPw() {
  int speedError = mFpsRight - targetMFpsRight;
  int pwCorrection = MOTOR_GAIN_DW * ((float) speedError);
  int pw =  targetPwDw - pwCorrection;
  
  if (pw > 65535) pw = 65535;
  else if (pw < 0) pw = 0;

  if (pw > ZERO_PW) {
    g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_DIR].ulPin; // HIGH
    pw = (pw - ZERO_PW) * 2.0;
  } else {
    g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_DIR].ulPin;   // LOW
    pw = (ZERO_PW - PW) * 2.0;
  }
  if (mode != MODE_PWM_SPEED) {
    pwm_write_duty(MOT_RIGHT_PWMH, pw);
  }
}


void readSpeedRight() {
  //  noInterrupts();
  long sum = wsMFpsRightSum;
  int count =  wsMFpsRightCount;
  wsMFpsRightSum = 0L;
  wsMFpsRightCount = 0;
  //  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeRight);
    if (newMFps < mFpsRight) mFpsRight = newMFps; // Set new if lower
  }
  else {
    mFpsRight = sum / count;
  }
  fpsRight = ((double) mFpsRight) / 1000.0;
}

void readSpeedLeft() {
  //  noInterrupts();
  long sum = wsMFpsLeftSum;
  int count =  wsMFpsLeftCount;
  wsMFpsLeftSum = 0L;
  wsMFpsLeftCount = 0;
  //  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeLeft);
    if (newMFps < mFpsLeft) mFpsLeft = newMFps; // Set new if lower
  }
  else {
    mFpsLeft = sum / count;
  }
  fpsLeft = ((double) mFpsLeft) / 1000.0;
}



void readSpeed() {
  readSpeedRight();
  readSpeedLeft();
  wheelSpeedFps = (fpsLeft + fpsRight) / 2.0D;
  mWheelSpeedFps = (mFpsRight + mFpsLeft) / 2.0D;
}
************************************************************************************************/

