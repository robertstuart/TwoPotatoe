
//const int BATTERY_WARNING = 1090;  // about 10% capacity (centivolts)
//const int BATTERY_CRITICAL = 1000; // about 1% cap (centivolts)
const int BATTERY_WARNING = 726;  // about 10% capacity (centivolts)
const int BATTERY_CRITICAL = 666; // about 1% cap (centivolts)

int beepCycleCount = 0;
boolean beepStat = false;
int *beepSequence;
int beepPtr = 0;

#define SONAR_BUF_SIZE 6
int sonarBufRight[SONAR_BUF_SIZE];
int sonarBufPtrRight = 0;
int sonarBufLeft[SONAR_BUF_SIZE];
int sonarBufPtrLeft = 0;

boolean flip = false;
int warningCount = 0;
int criticalCount = 0;
int addFlip = 0;
boolean redLedState = false;
boolean greenLedState = false;
unsigned long taskMilliseconds = 0L;
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
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  readXBee();  // Read commands from PC or Hand Controller
  readBluetooth();
  readSonar();
  //  motorIdle();
  led();
  battery();
  controllerConnected();
  liftJump();
  setRunningState();
  gyroTemperature();
}


/**************************************************************************.
 *
 * setRunningState()
 *
 *     Set the TP_STATE_RUNNING bit if the following are true:
 *         TP_STATE_RUN_READY is true
 *         TP_STATE_UPRIGHT is true
 *         TP_STATE_ON_GROUND or isJump is true
 *         TP_STATE_MOTOR_FAULT ????
 *
 *      Set x and y to zero if there is no connection to
 *      a controller or STATE_MOTOR_FAULT is true.
 *
 *      Set blinking according to the above states.
 *
 **************************************************************************/
void setRunningState() {

  if (mode == MODE_TP6) {
    // Set the runnng bit to control motors
    if ((isRunReady && isUpright) && (!isLifted)) {
      isRunning = true;
    }
    else {
      isRunning = false;
    }
  }
  else { // For all test modes, just set accoding to ready bit
    isRunning = isRunReady;
  }

  // Set the blue connection led
  if (isHcActive) setBlink(BLUE_LED_PIN, BLINK_ON);
  else if (isPcActive) setBlink(BLUE_LED_PIN, BLINK_SB);
  else setBlink(BLUE_LED_PIN, BLINK_FF);

  // set red (mode) and yellow (state)
  switch (mode) {
    case MODE_TP6:
      if  (isRouteInProgress) {
        setBlink(YELLOW_LED_PIN, BLINK_FF);
        setBlink(RED_LED_PIN, BLINK_FF);
      }
      else {
        setBlink(RED_LED_PIN, BLINK_SF);
        if (isRunning)         setBlink(YELLOW_LED_PIN, BLINK_ON);
        else if (isRunReady)   setBlink(YELLOW_LED_PIN, BLINK_FF);
        else                   setBlink(YELLOW_LED_PIN, BLINK_SF);
      }
      if ((!isHcActive) && (!isPcActive)) {
        pcX = pcY = hcX = hcY = 0.0f;
      }
      break;
    default:
      setBlink(RED_LED_PIN, BLINK_SF);
      if (isRunReady) setBlink(YELLOW_LED_PIN, BLINK_SF);
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
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

  boolean cState = ((abs(gaFullPitch) < 45.0) && ((abs(gaRoll) < 45))); // Current real state
  if (cState != tState) {
    tTime = timeMilliseconds; // Start the timer for a state change.
    tState = cState;
  }
  else {
    if ((timeMilliseconds - tTime) > 50) {
      isUpright = cState;
    }
  }
}  // End safeAngle().


/**************************************************************************.
 * liftJump() Set the isLift  variable.
 *            isLift is true if onGround has been false > XX seconds.
 **************************************************************************/
void liftJump() {
  static unsigned long liftTimer = 0UL; // Zero if not set.
  static boolean isOldOnGround = false;
  
  forceLeft = analogRead(L_FORCE_PIN);
  forceRight = analogRead(R_FORCE_PIN);
  isOnGround = (forceLeft < 800) && (forceRight < 950);

  if (!isOnGround) {
    // TP has just left the ground.
    if (isOldOnGround) {  // Just left the ground?
      isStepFall = true; // Used by jump().
      liftTimer = timeMilliseconds + 400;  // .4 sec
    } 
    else { // In the air.
      if (timeMilliseconds > liftTimer) { // In air > .4 sec?
        isLifted = true;
      }
    }
  }
  else { // On the ground.    
    isLifted = false;
    liftTimer = 0UL;
  }
  isOldOnGround = isOnGround;
}


/**************************************************************************.
 *
 * controllerConnected()
 *
 *********************************************************/
void controllerConnected() {
  isHcActive =  ((tHc + 1000) > timeMilliseconds) ? true : false;
  isPcActive =  ((tPc + 1000) > timeMilliseconds) ? true : false;
}



/**************************************************************************.
 * battery()
 ***************************************************************/
void battery() {
  static unsigned long batteryTrigger = 0L;
  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger = timeMilliseconds + 1000;  // 1 per second
    battVolt = (1000 * analogRead(BATT_PIN)) / 451;
  }
}



/**************************************************************************.
 * gyroTemperature() Not used. Doesn't improve drift over small 
 *                   temperature ranges.
 **************************************************************************/
void gyroTemperature() {
  static unsigned long gyroTemperatureTrigger = 0UL;
  if (timeMilliseconds > gyroTemperatureTrigger) {
    gyroTemperatureTrigger = timeMilliseconds + 1000;  // 1 per second
    int t = gyro.readReg(L3G::OUT_TEMP);
    if (t > 127) t -= 256;
    gyroFahrenheit = (((33 - t) * 9) / 5) + 32;
  }
}


/**************************************************************************.
 *  led() Call at least 10/sec
 **************************************************************************/
void led() {
  static unsigned long blinkTrigger = 0L;
  if (timeMilliseconds > blinkTrigger) {
    blinkTrigger = timeMilliseconds + 100;  // 10 per second

    int b = (patternBlue[blinkPtrBlue++] == 1) ? HIGH : LOW;
    if (patternBlue[blinkPtrBlue] == END_MARKER) blinkPtrBlue = 0;
    digitalWrite(BLUE_LED_PIN, b);
digitalWrite(GREEN_LED_PIN, b);
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
  static int buTrigger = 0;
  static boolean buState = false;
  String s = "";
  if (digitalRead(YE_SW_PIN) == LOW) {
    if (timeMilliseconds > yeTrigger) {
      yeTrigger = timeMilliseconds + 1000;
      isRunReady = isRunReady ? false : true;
    }
  }
  if (digitalRead(BU_SW_PIN) == LOW) {
    if (timeMilliseconds > buTrigger) {
      buTrigger = timeMilliseconds + 1000;
      buState = !buState;
    }
  }
}



/**************************************************************************.
 * getControllerXY() return fps from two controllers & hold state
 **************************************************************************/
void setControllerXY() {
  static double y = 0.0D;
  static double x = 0.0D;

//  if (!isHoldFps) {
//    if (abs(hcY) > abs(pcY)) y = hcY;
//    else y = pcY;
//  }
//  controllerY = y;
//
//  if (!isHoldHeading) {
//    if (abs(hcX) > abs(pcX)) x = hcX;
//    else x = pcX;
//  }
//  controllerX = x;
  controllerX = hcX;
  controllerY = hcY;
}



/**************************************************************************.
 * getControllerX() return turn from two controllers & hold state
 **************************************************************************/
double getControllerX() {
  //  return hcX;
  static double x = 0.0D;
  if (!isHoldHeading) {
    if (abs(hcX) > abs(pcX)) x = hcX;
    else x = pcX;
  }
  return x;
}



/**************************************************************************.
 * readSonar() 
 **************************************************************************/
void readSonar() {
  const int S_BUFFER_SIZE = 20;
  static char msgStr[S_BUFFER_SIZE + 1];
  static boolean isMsgInProgress = false;
  static int msgPtr = 0;
  static boolean isRight = false;
  
  float distance = 0.0;
 
  while (SONAR_SER.available()) {
    byte b = SONAR_SER.read();
    if (isMsgInProgress) {
      if (b == 13) {
        msgStr[msgPtr] = 0;
        int e = sscanf(msgStr, "%f", &distance);
        if ((e > 0) && (distance < 30.0)) {
          doSonar(distance, isRight);
        }
        isMsgInProgress = false;
      }
      else if (msgPtr >= S_BUFFER_SIZE) {
        isMsgInProgress = false;
      } else {
        msgStr[msgPtr++] = b;
      }
    } else {
      if (b == 'R') isRight = true;
      else if (b == 'L') isRight = false;
      else continue; 
      msgPtr = 0;
      isMsgInProgress = true;         
    }
  }
}



/**************************************************************************.
 * doSonar() Process received distance from sonar.
 *           Called for every sonar reading.
 **************************************************************************/
void doSonar(float distance, int isRight) {
    
  // Collect data for least squares calculation.
  if (isGXAxis) {
    lsXArray[lsPtr] = (float) currentMapLoc.x;
    lsYLArray[lsPtr] = (float) currentMapLoc.y;
  } else {
    lsXArray[lsPtr] = (float) currentMapLoc.y;
    lsYLArray[lsPtr] = (float) currentMapLoc.x;
  }
  lsSArray[lsPtr] = (float) distance;
  if (lsPtr < (LS_ARRAY_SIZE - 1)) lsPtr++;

  // Collect data for Charted Object measurements. 
  if (isRight) {     
    sonarRightArray[sonarRightArrayPtr] = sonarRight;
    sonarRightArrayPtr = ++sonarRightArrayPtr % SONAR_ARRAY_SIZE;
    sonarRight = distance;
  } else {
    sonarLeftArray[sonarLeftArrayPtr] = sonarLeft;
    sonarLeftArrayPtr = ++sonarLeftArrayPtr % SONAR_ARRAY_SIZE;
    sonarLeft = distance;
  }

  if (isRouteInProgress) routeLog();
}

void setSonar(int mode) {
  int r, l;
  sonarMode = mode;
  if (mode == SONAR_RIGHT) {
    r = HIGH;
    l = LOW;
  } else if (mode == SONAR_LEFT) {
    r = LOW;
    l = HIGH;
  } else if (mode == SONAR_BOTH) {
    r = HIGH;
    l = HIGH;
  } else {
    r = LOW;
    l = LOW;
  }
  digitalWrite(SONAR_RIGHT_PIN, r);
  digitalWrite(SONAR_LEFT_PIN, l);
}

/**************************************************************************.
 *  rangeAngle() Set angle value between -180 and +180
 **************************************************************************/
double rangeAngle(double head) {
  while (head > 180.0D) head -= 360.0D;
  while (head <= -180.0D) head += 360.0D;
  return head;
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
//
//boolean isBitClear(int test, byte b) {
//  return ((test & b) == 0);
//}
//
//boolean isBitSet(int test, byte b) {
//  return ((test & b) != 0);
//}
