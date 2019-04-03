// defines for encoder pins. Reverse to change direction.
const int ENCA_RIGHT_PIN = 26;  
const int ENCB_RIGHT_PIN =  27;   
const int ENCA_LEFT_PIN =   28;
const int ENCB_LEFT_PIN =   29; 

const int PWM_RIGHT_PIN =    5; 
const int DIR_RIGHT_PIN =    6;   
const int PWM_LEFT_PIN =     3;  
const int DIR_LEFT_PIN =     4; 

void motorInit() {
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(ENCA_RIGHT_PIN, INPUT);
  pinMode(ENCA_LEFT_PIN, INPUT);
  pinMode(ENCB_RIGHT_PIN, INPUT);
  pinMode(ENCB_LEFT_PIN, INPUT);

  analogWriteFrequency(PWM_RIGHT_PIN, 20000);
  analogWriteFrequency(PWM_LEFT_PIN, 20000);

  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);
  analogWrite(PWM_LEFT_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT_PIN), encoderIsrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT_PIN), encoderIsrLeft, CHANGE);
}

// Use digitalWrite/ReadFast()? ????????????????????????????????????????????????????

/**************************************************************************.
 * encoderIsr???()
 **************************************************************************/
void encoderIsrRight() {
  static boolean encAStat;
  static boolean encBStat;
  
  boolean encA = (digitalRead(ENCA_RIGHT_PIN)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsRight++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned int lastTickTime = tickTimeRight;
  tickTimeRight = micros();
  boolean encB = (digitalRead(ENCB_RIGHT_PIN)) ? true : false;
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
  boolean encA = (digitalRead(ENCA_LEFT_PIN)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsLeft++;
    return;
  }
  encAStat = encA;
  unsigned int lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  boolean encB = (digitalRead(ENCB_LEFT_PIN)) ? true : false;
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
//const int DEAD_ZONE = 1600; // This pw or less gives zero fps.
const int DEAD_ZONE = 5.0; // This pw or less gives zero fps.
//const float FPS_TO_PW = 2900.0; // change in PW gives change of 1.0 FPS, 775 24V motor
const float FPS_TO_PW = 4.9; // change in PW gives change of 1.0 FPS, 775 18V motor
/**************************************************************************.
 * checkMotor????()  Called every loop.
 *                    Read the speed & set the motor pw.
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
  bool isFwd = (wsTarget > 0.0);
  setMotorRight(pw, isFwd);
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
  bool isFwd = (wsTarget > 0.0);
  setMotorLeft(pw, isFwd);
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
    checkMotorRight();
    checkMotorLeft();
    wFps = (wFpsLeft + wFpsRight) / 2.0;
    wMFps = (wMFpsRight + wMFpsLeft) / 2;
}



/*********************************************************
 *  setMotor???() Set the pw and direction of the motor.
 *********************************************************/
void setMotorRight(unsigned int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;  
   digitalWrite(DIR_RIGHT_PIN, !isFwd ? HIGH : LOW);   
   analogWrite(PWM_RIGHT_PIN, pw);
}
void setMotorLeft(unsigned int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  if (pw > 255) pw = 255;

  digitalWrite(DIR_LEFT_PIN, isFwd ? HIGH : LOW);   
  analogWrite(PWM_LEFT_PIN, pw); 
}
