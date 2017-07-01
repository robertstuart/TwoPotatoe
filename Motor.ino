const int MOTOR_PW_MAX = 65535;


void motorInitTp() {
  // Initialize the pwm frequency
  pwm_set_resolution(16);
  pwm_setup(MOT_RIGHT_PWMH, TP_PWM_FREQUENCY, 1);  // on clock A
  pwm_setup(MOT_LEFT_PWMH, TP_PWM_FREQUENCY, 1);  // on clock A

  // Set motor pins & states
  pinMode(MOT_RIGHT_DIR, OUTPUT);
  pinMode(MOT_LEFT_DIR, OUTPUT);
  setMotorRight(0, FWD);
  setMotorLeft(0, FWD);

  attachInterrupt(MOT_RIGHT_ENCA, encoderIsrRight, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeft, CHANGE);
}

/**************************************************************************.
 * encoderIsr???()
 **************************************************************************/
void encoderIsrRight() {
  static boolean encAStat;
  static boolean encBStat;
  
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsRight++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned int lastTickTime = tickTimeRight;
  tickTimeRight = micros();
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
  if (encB == encBStat) return;  // Ignore reversal of direction
  encBStat = encB;
  
  // Set the speed & tickPosition
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickPositionRight++;
  }
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickPositionRight--;
  }
  int intrMFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  intrMFpsRightSum += intrMFpsRight;
  intrMFpsRightCount++;
} // encoderIsrRight()


/**************************************************************************.
 * encoderIsrLeft()
 **************************************************************************/
void encoderIsrLeft() {
  static boolean encAStat;
  static boolean encBStat;
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsLeft++;
    return;
  }
  encAStat = encA;
  unsigned int lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  if (encB == encBStat) return;  // Ignore reversal of direction.
  encBStat = encB;
  
  // Set the speed & tickPosition
  if (encA == encB) {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickPositionLeft--;
  }
  else {
    tickPeriodLeft = (long)tickTimeLeft - (long) lastTickTime;
    tickPositionLeft++;
  }
  int intrMFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  intrMFpsLeftSum += intrMFpsLeft;
  intrMFpsLeftCount++;
} // end encoderIsrLeft();



//const float MOTOR_GAIN = 1.0;
//const float MOTOR_GAIN = 2.0;
const float MOTOR_GAIN = 5.0;
const int DEAD_ZONE = 1600; // This pw or less gives zero fps.
const float FPS_TO_PW = 2900.0; // change in PW gives change of 1.0 FPS, 775 24V motor
/**************************************************************************.
 * checkMotor????()  Called 10,000 time/sec in at timer interupt
 *                   service routine.
 **************************************************************************/
void checkMotorRight() {
  float motorGain = MOTOR_GAIN;
  readSpeedRight();

  float wsError = (float) (targetWFpsRight - wFpsRight);       // Wheel speed error
  if (abs(targetWFpsRight) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWFpsRight) * 8.0);
  }
  float wsTarget = targetWFpsRight + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(wsTarget * FPS_TO_PW) + DEAD_ZONE;            // Pw for the target.
  if (pw <= DEAD_ZONE) pw = 0; 
  int direction = (wsTarget > 0.0) ? FWD : BKWD;
  if ((mode == MODE_2P) || (mode == MODE_TP_SPEED)) {
    setMotorRight(pw, direction);
  }
}

void checkMotorLeft() {
  float motorGain = MOTOR_GAIN;
  readSpeedLeft();

  float wsError = (float) (targetWFpsLeft - wFpsLeft);       // Wheel speed error
  if (abs(targetWFpsLeft) < 0.5) {
    motorGain = 1.0 + (abs(targetWFpsLeft) * 8.0);
  }
  float wsTarget = targetWFpsLeft + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(wsTarget * FPS_TO_PW) + DEAD_ZONE;            // Pw for the target.
  if (pw <= DEAD_ZONE) pw = 0; 
  int direction = (wsTarget > 0.0) ? FWD : BKWD;
  if ((mode == MODE_2P) || (mode == MODE_TP_SPEED)) {
    setMotorLeft(pw, direction);
  }
}



/**************************************************************************.
 * readSpeed????()  Called every 1000usec from CheckMotor???
 **************************************************************************/
void readSpeedRight() {
  //  noInterrupts();
  int sum = intrMFpsRightSum;
  int count =  intrMFpsRightCount;
  intrMFpsRightSum = 0L;
  intrMFpsRightCount = 0;
  //  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeRight);
    if (newMFps > 0) {
      if (newMFps < wMFpsRight) wMFpsRight = newMFps; // Set new if lower
    } else {
      if (newMFps > wMFpsRight) wMFpsRight = newMFps; // Set new if lower
    } 
  }
  else {
    wMFpsRight = sum / count;
  }
  wFpsRight = (wMFpsRight) / 1000.0;
}

void readSpeedLeft() {
  //  noInterrupts();
  long sum = intrMFpsLeftSum;
  int count =  intrMFpsLeftCount;
  intrMFpsLeftSum = 0L;
  intrMFpsLeftCount = 0;
  //  interrupts();
  if (count == 0) {
    int newMFps = ENC_FACTOR_M / (micros() - tickTimeLeft);
    if (newMFps > 0) {
      if (newMFps < wMFpsLeft) wMFpsLeft = newMFps; // Set new if lower
    } else {
      if (newMFps > wMFpsLeft) wMFpsLeft = newMFps; // Set new if lower
    }
  }
  else {
    wMFpsLeft = sum / count;
  }
 wFpsLeft = ((float) wMFpsLeft) / 1000.0;
}



void checkMotors() {
  static unsigned int pollCount;
//  if (!(++pollCount % 10)) {  // Do every 10th call (1000/sec)
    checkMotorRight();
    checkMotorLeft();
    wFps = (wFpsLeft + wFpsRight) / 2.0;
    wMFps = (wMFpsRight + wMFpsLeft) / 2;
//    isNewCheck = true;
//        addLog(
//        (long) timeMilliseconds,
//        (short) (0.0 * 100.0),
//        (short) (wFps * 100.0),
//        (short) (0.0 * 100.0),
//        (short) (0.0 * 100.0),
//        (short) (0.0 * 100.0),
//        (short) (0.0 * 100.0)
//   );

//  }
}



/*********************************************************
 *  setMotor???() Set the pw and direction of the motor.
 *                The pw of the motor is between zero and
 *                65535.  
 *********************************************************/
void setMotorRight(int pw, int direction) {
  if (!isRunning) pw = 0;
  if (pw > 62000) pw = 65535; // Ignore dead area; 

  if (direction == FWD) {
      g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_DIR].ulPin; // HIGH    
  } else {
      g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_DIR].ulPin;   // LOW
  }
  pwm_write_duty(MOT_RIGHT_PWMH, pw);
}
void setMotorLeft(int pw, int direction) {
  if (!isRunning) pw = 0;

  if (pw > 62000) pw = 65535; // Ignore dead area; 

  if (direction == FWD) {
      g_APinDescription[MOT_LEFT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_DIR].ulPin; // HIGH    
  } else {
      g_APinDescription[MOT_LEFT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_DIR].ulPin;   // LOW
  }
  pwm_write_duty(MOT_LEFT_PWMH, pw); 
}
