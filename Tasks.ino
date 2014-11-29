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
unsigned long onGroundTime = 0L;
unsigned long warningTrigger = 0;
unsigned long batteryLastGood = 0;


const byte* patternBlue = BLINK_OFF;       
const byte* patternYellow = BLINK_OFF;       
const byte* patternRed = BLINK_OFF;       
int blinkPtrBlue = 0;
int blinkPtrYellow = 0;
int blinkPtrRed = 0;


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
  readXBee();  // Read commands from PC or Hand Controller
  motorIdle();
  led();
  battery();
  controllerConnected();
  setRunningState();
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
}


/**************************************************************************.
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
 **************************************************************************/
void setRunningState() {

  // Set the runnng bit
  if (     ((tpState & TP_STATE_RUN_READY) != 0)
           && ((tpState & TP_STATE_UPRIGHT) != 0)
           && ((tpState & TP_STATE_ON_GROUND) != 0)) {
    tpState = tpState | TP_STATE_RUNNING;
  }
  else {
    tpState = tpState & ~TP_STATE_RUNNING;
  }
  
  // Set the blue connection led
  if (isStateBit(TP_STATE_HC_ACTIVE)) setBlink(BLUE_LED_PIN, BLINK_ON);
  else if (isStateBit(TP_STATE_PC_ACTIVE)) setBlink(BLUE_LED_PIN, BLINK_SB);
  else setBlink(BLUE_LED_PIN, BLINK_FF);

  // set x and y if no controller connected
  switch (mode) {
  case MODE_TP5:
    setBlink(RED_LED_PIN, BLINK_SB);    
    if (isStateBit(TP_STATE_RUNNING)) setBlink(YELLOW_LED_PIN, BLINK_ON);
    else if (isStateBit(TP_STATE_RUN_READY)) setBlink(YELLOW_LED_PIN, BLINK_FF);
    else setBlink(YELLOW_LED_PIN, BLINK_SF);
    if (!(isStateBit(TP_STATE_HC_ACTIVE) | isStateBit(TP_STATE_PC_ACTIVE))) {
      controllerY = 0.0f;
      controllerX = 0.0f;
    }
    break;
  case MODE_POSITION: 
    setBlink(RED_LED_PIN, BLINK_SF);    
    if (isStateBit(TP_STATE_RUN_READY)) setBlink(YELLOW_LED_PIN, BLINK_SF);
    else setBlink(YELLOW_LED_PIN, BLINK_ON);
    break;
  default:
    setBlink(RED_LED_PIN, BLINK_SF);    
    if (isStateBit(TP_STATE_RUN_READY)) setBlink(YELLOW_LED_PIN, BLINK_SF);
    else setBlink(YELLOW_LED_PIN, BLINK_ON);
    break;
  }
  
  
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
        setStateBit(TP_STATE_UPRIGHT, true);
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright
  
  boolean sState = isStateBit(TP_STATE_UPRIGHT);
  boolean cState = ((abs(gaPitch) < 45.0) && ((abs(gaRollAngle) < 35)));
//sState = isStateBitSet(TP_STATE_UPRIGHT);
//cState = ((abs(gaPitchAngle) < 45.0) && ((abs(gaRollAngle) < 35)));
  if (cState != tState) {
    tTime = timeMilliseconds; // Start the timer for a state change.
    tState = cState;
  }
  else {
    if ((timeMilliseconds - tTime) > 50) {
      // We have a persistent state change
      if (sState != cState) {
        // This is a state change.
        setStateBit(TP_STATE_UPRIGHT, cState);
        if (!cState) { // fallen?
//          sendDumpData();
        }
      }
    } 
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


/**************************************************************************.
 *  led() Call at least 10/sec
 **************************************************************************/
void led() {
  if (timeMilliseconds > blinkTrigger) {
    blinkTrigger += 100;  // 10 per second
    
    int b = (patternBlue[blinkPtrBlue++] == 1) ? HIGH : LOW;
    if (patternBlue[blinkPtrBlue] == END_MARKER) blinkPtrBlue = 0;
    digitalWrite(BLUE_LED_PIN, b);
    b = (patternYellow[blinkPtrYellow++] == 1) ? HIGH : LOW;
    if (patternYellow[blinkPtrYellow] == END_MARKER) blinkPtrYellow = 0;
    digitalWrite(YELLOW_LED_PIN, b);
    b = (patternRed[blinkPtrRed++] == 1) ? HIGH : LOW;
    if (patternRed[blinkPtrRed] == END_MARKER) blinkPtrRed = 0;
    digitalWrite(RED_LED_PIN, b);
  }
}

/**************************************************************************.
 *  setBlink() Set blink patter for led
 **************************************************************************/
void setBlink(int led, byte* pattern) {
  switch (led) {
    case BLUE_LED_PIN:
      if (patternBlue != pattern) {
        patternBlue = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
      }
      break;
    case YELLOW_LED_PIN:
      if (patternYellow != pattern) {
        patternYellow = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
      }
      break;
    case RED_LED_PIN:
      if (patternRed != pattern) {
        patternRed = pattern;
        blinkPtrBlue = 0;
        blinkPtrYellow = 0;
        blinkPtrRed = 0;
      }
      break;
    default:
      break;
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

/**************************************************************************.
 * switches()
 *      Toggle TP_STATE_RUN_READY on yellow switch.  1 sec dead period.
 **************************************************************************/
void switches() {
  static int yeTrigger = 0;
  if (digitalRead(YE_SW_PIN) == LOW) {
    if (timeMilliseconds > yeTrigger) {
      yeTrigger = timeMilliseconds + 1000;
      setStateBit(TP_STATE_RUN_READY, !isStateBit(TP_STATE_RUN_READY));
    }
  }
}

/**************************************************************************.
 *  addLog() Put values in the dump arrays.
 **************************************************************************/
void addLog(long aVal, short bVal, short cVal, short dVal, short eVal, short fVal, short gVal) {
  if (!isDumpingData) {
    if (aVal == 0L) aVal = 1L; // don't indicate end
    aArray[dataArrayPtr] = aVal;
    bArray[dataArrayPtr] = bVal;
    cArray[dataArrayPtr] = cVal;
    dArray[dataArrayPtr] = dVal;
    eArray[dataArrayPtr] = eVal;
    fArray[dataArrayPtr] = fVal;
    gArray[dataArrayPtr] = gVal;
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

boolean isBitSet(int test, byte b) {
  return ((test & b) != 0);
}
boolean isStateBit(byte b) {
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

