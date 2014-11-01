#include <DueTimer.h>

//const int BATTERY_WARNING = 1090;  // about 10% capacity (centivolts)
//const int BATTERY_CRITICAL = 1000; // about 1% cap (centivolts)
const int BATTERY_WARNING = 726;  // about 10% capacity (centivolts)
const int BATTERY_CRITICAL = 666; // about 1% cap (centivolts)

int beepCycleCount = 0;
boolean beepStat = false;
int *beepSequence;
int beepPtr = 0;

boolean flip = false;
int warningCount = 0;
int criticalCount = 0;
int addFlip = 0;
boolean redLedState = false;
boolean greenLedState = false;
unsigned long taskMilliseconds = 0L;
unsigned long batteryTrigger = 0L;
unsigned long blinkTrigger = 0L;
unsigned long gravityTrigger = 0L;
//unsigned long audioTrigger = 0L;
unsigned long errorTrigger = 0L;
unsigned int taskPtr = 0;
unsigned int pingTpHCCount = 0;
unsigned long uprightTime = 0L;
unsigned long onGroundTime = 0L;
unsigned long warningTrigger = 0;
unsigned long batteryLastGood = 0;


/**************************************************************************.
 *
 * commonTasks()
 *
 *    Execute all tasks common to every algorithm.  This includes:
 *      1. Reading the XBee for commands.
 *      2. Flushing the serial output buffer
 *      3. Setting the time variables.
 *      4. Dumping any data if we are in "dump data" mode.
 *      5. Turning the power off if the motors have been idle.
 *      6. Flash the led.
 *
 **************************************************************************/
void commonTasks() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  readXBee();  // Read commands from PC or Hand Controller
  motorIdle();
  led();
  battery();
  controllerConnected();
  setRunningState();
}


/*********************************************************
 *
 * setRunningState()
 *
 *     Set the TP_STATE_RUNNING bit if the following are true:
 *         TP_STATE_RUN_READY is true
 *         TP_STATE_UPRIGHT is true
 *         TP_STATE_ON_GROUND is true
 *         TP_STATE_MOTOR_FAULT ????
 *
 *      Set x and y to zero if there is no connection to
 *      a controller or STATE_MOTOR_FAULT is true.
 *
 *      Set blinking according to the above states.
 *
 *********************************************************/
void setRunningState() {
  byte* blinkState = BLINK_SBYR;

  // Set the bit
  if (     ((tpState & TP_STATE_RUN_READY) != 0)
           && ((tpState & TP_STATE_UPRIGHT) != 0)
           && ((tpState & TP_STATE_ON_GROUND) != 0)) {
    //    if (!(tpState & TP_STATE_RUNNING)) { // only zero for state change
    tpState = tpState | TP_STATE_RUNNING;
    //      if (!isRouteInProgress) {
    //        tickDistanceRight = 0;
    //        tickDistanceLeft = (long) (magHeading * TICKS_PER_YAW_DEGREE);
    //        getTp5Angle();
    //        targetHeading = tickHeading;
    //      }
    //    }
  }
  else {
    tpState = tpState & ~TP_STATE_RUNNING;
  }

  // set x, y, and blink state if no controller connected
  if (!(tpState & (TP_STATE_HC_ACTIVE | TP_STATE_PC_ACTIVE))) {
    controllerY = 0.0f;
    controllerX = 0.0f;
    blinkState = BLINK_SBYR;  // Slow blinking if no connection
  }
  else {
    switch (mode) {
    case MODE_TP5:
      if (isStateBitClear(TP_STATE_RUNNING)  && isStateBitSet(TP_STATE_RUN_READY)) {
        blinkState = BLINK_FRB;
      }
      else blinkState = BLINK_B_FR;
      break;
    case MODE_TP6: 
      if (isStateBitClear(TP_STATE_RUNNING)  && isStateBitSet(TP_STATE_RUN_READY)) {
        blinkState = BLINK_FRB;
      }
      else blinkState = BLINK_B_FY;
      break;
    default:
      blinkState = BLINK_FYR; // some testing mode
    }
  }
  setBlink(blinkState);
}


/*********************************************************
 *
 * safeAngle()
 *
 *     Check to see if we have fallen sidways or forwards.
 *     If so, unset the STATE_UPRIGHT bit.
 *     Otherwise, set the bit.
 *
 *********************************************************/
void safeAngle() {
  if ((abs(gaPitchAngle) > 45.0) || ((abs(gaRollAngle) > 35))) {  // Not at a safe angle?
    if (timeMilliseconds > (uprightTime + 50)) { // more that 1/20th of a second?
      tpState = tpState & ~TP_STATE_UPRIGHT;
    }
  }
  else {
    tpState = tpState | TP_STATE_UPRIGHT;
    uprightTime = timeMilliseconds;
  }
}  // End safeAngle().


/*********************************************************
 *
 * gravity()
 *
 *     Check to see if we sitting on the ground.
 *     If so, set the STATE_UPRIGHT bit.
 *     Otherwise, unset the bit.
 *
 *********************************************************/
void gravity() {
  pressure = analogRead(PRESSURE_PIN);
  if (pressure > 300) {  // Not on ground?
    if (timeMilliseconds > (onGroundTime + 100)) {  // more that 1/10 of a second?
      tpState = tpState & ~TP_STATE_ON_GROUND;
    }
  }
  else {
    tpState = tpState | TP_STATE_ON_GROUND;
    onGroundTime = timeMilliseconds;
  }
}

/*********************************************************
 *
 * controllerConnected()
 *
 *********************************************************/
void controllerConnected() {
  byte hcBit =  ((tHc + 1000) > timeMilliseconds) ? TP_STATE_HC_ACTIVE : 0;
  byte pcBit =  ((tPc + 1000) > timeMilliseconds) ? TP_STATE_PC_ACTIVE : 0;
  byte tmp = tpState & (~(TP_STATE_HC_ACTIVE | TP_STATE_PC_ACTIVE)); // Clear bits
  tpState = tmp | hcBit | pcBit;
}



/***************************************************************
 *
 * battery()
 *
 *       check battery, set batteryVoltage, turn off if too low
 *
 ***************************************************************/
void battery() {

  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger += 1000;  // 1 per second
    bmBattVolt = (1000 * analogRead(MB_BATT_PIN)) / 455;
    emBattVolt = ((1000 * analogRead(EB_BATT_PIN)) / 446) - bmBattVolt;
    lBattVolt = (1000 * analogRead(L_BATT_PIN)) / 455;

    //    // Check for warning condition.
    //    if (batteryVolt < BATTERY_WARNING) {
    //      // Continuously low for a second?
    //      if ((batteryLastGood + 1000) < timeMilliseconds) {
    //        if (timeMilliseconds > warningTrigger) {
    //          beep(BEEP_WARBLE);
    //          warningTrigger += 60000;
    //        }
    //      }
    //    }
    //    else {
    //      batteryLastGood = timeMilliseconds;
    //    }
    //
    //    // Check for critical condition.
    //    if (batteryVolt < BATTERY_CRITICAL) {
    //      if ((timeMilliseconds + 10000) > warningTrigger) {
    //        digitalWrite(PWR_PIN, LOW);  // Power down TwoPotatoe
    //      }
    //    }
  } // end batteryTrigger
}

// Set the blink pattern
void setBlink(byte* pattern) {
  if (pattern != blinkPattern) {
    blinkPattern = pattern;
    blinkPtr = 0;
  }
}



/***************************************************************
 *
 * compass()  Read the compass
 *
 ***************************************************************/
void myCompass() {

}

void motorIdle() {
  static unsigned long lastActiveTime;
  static int rMotor = 42;
  static int lMotor = 42;
  int rMotorTmp = digitalRead(MOT_RIGHT_ENCA);
  int lMotorTmp = digitalRead(MOT_LEFT_ENCA);
  if ((rMotorTmp != rMotor) || (lMotorTmp != lMotor)) {
    lastActiveTime = timeMilliseconds;
    rMotor = rMotorTmp;
    lMotor = lMotorTmp;
  }
  if ((timeMilliseconds - lastActiveTime) > (1000 * 60 * 10)) digitalWrite(PWR_PIN, LOW);
}


/***************************************************************
 *
 * led()  Call at least 10/sec
 *
 ***************************************************************/
void led() {
  if (timeMilliseconds > blinkTrigger) {
    int b = LOW, y = LOW, r = LOW;
    blinkTrigger += 100;  // 10 per second
    byte blink = blinkPattern[blinkPtr++];
    if (blinkPattern[blinkPtr] == END_MARKER) {
      blinkPtr = 0;
    }

    if (blink & 1) {
      b = HIGH;
    }
    if (blink & 2) {
      y = HIGH;
    }
    if (blink & 4) {
      r = HIGH;
    }

    digitalWrite(BLUE_LED_PIN, b);
  digitalWrite(MOTOR_RESET_PIN, b);
    digitalWrite(YELLOW_LED_PIN, y);
    digitalWrite(RED_LED_PIN, r);
  }
}





void beep(int seq[]) {
  beepPtr = 0;
  beepSequence = seq;
  Timer3.attachInterrupt(beepIsr);
  setBeep();
}

void setBeep() {
  int freq = beepSequence[beepPtr];
  if (freq != 0) {
    int halfCycle = (1000000 / 2) / freq;
    int dur = beepSequence[beepPtr + 1];
    beepCycleCount = (dur * 1000) / halfCycle;
    Timer3.start(halfCycle);
    beepPtr += 2;
  }
  else {
    Timer3.stop();
    Timer3.detachInterrupt();
    digitalWrite(SPEAKER_PIN, LOW);
  }
}

void beepIsr() {
  if (--beepCycleCount <= 0) {
    setBeep();
  }
  beepStat = !beepStat;
  digitalWrite(SPEAKER_PIN, beepStat);
}

void runSwitch() {
  if (digitalRead(YE_SW_PIN) == LOW) {
    setStateBit(TP_STATE_RUN_READY, true);
  }
}

/************************************************************************
 *  runLog() Put values in the dump arrays.
 ************************************************************************/
void runLog(long aVal, long bVal, long cVal, long dVal) {
  if (!isDumpingData) {
    if (aVal == 0L) aVal = 1L; // don't indicate end
    aArray[dataArrayPtr] = aVal;
    bArray[dataArrayPtr] = bVal;
    cArray[dataArrayPtr] = cVal;
    dArray[dataArrayPtr] = dVal;
    dataArrayPtr++;
    dataArrayPtr = dataArrayPtr %  DATA_ARRAY_SIZE;
  }
}

void setStateBit(byte by, boolean bo) {
  if (bo) tpState = tpState | by;
  else tpState = tpState & (~by);
}

boolean isBitClear(int test, byte b) {
  return ((test & b) == 0);
}
boolean isStateBitClear(byte b) {
  return ((tpState & b) == 0);
}
boolean isBitSet(int test, byte b) {
  return ((test & b) != 0);
}
boolean isStateBitSet(byte b) {
  return ((tpState & b) != 0);
}

void debugFloat(char msg[], float f) {
  Serial.print(msg);
  Serial.println(f);
}
void debugInt(char msg[], int i) {
  Serial.print(msg);
  Serial.println(i);
}

