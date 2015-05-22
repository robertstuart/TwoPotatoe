
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
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  readXBee();  // Read commands from PC or Hand Controller
  readBluetooth();
//  readBluetooth();
//  motorIdle();
  led();
  battery();
  controllerConnected();
  onGround();
  setRunningState();
  sonar();
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
    if ((isRunReady && isUpright && isOnGround) || isJump) {
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
      controllerY = 0.0f;
      controllerX = 0.0f;
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
  
  boolean cState = ((abs(gaPitch) < 45.0) && ((abs(gaRoll) < 35))); // Current real state
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
 * onGround() set  T
 *
 *      The TP_STATE_ON_GROUND bit is set whenever both wheels show a force
 *      greater than the threshold.  Whenever the force on either wheels 
 *      drops below the threshold TP_STATE_ON_GROUND is cleared but  
 *      isJump is set to true.  After 0.3 seconds in the "off-ground"
 *      isJump is set to false.
 **************************************************************************/
void onGround() {
  int groundTime = 0;
  forceLeft = analogRead(L_FORCE_PIN);
  forceRight = analogRead(R_FORCE_PIN);
  isOnGround = (forceLeft < 700) && (forceRight < 700);
  if (isOnGround) {
    if (!isOnGround)  {
      if (magHeading == 0.0) return; // Wait until heading has been read.
      isOnGround = true;
      isJump = false;
      groundTime = timeMilliseconds;
    }
  }
  else { // in air
    isOnGround = false;
    if (timeMilliseconds < (groundTime + 300)) isJump = true;
    else isJump = false; 
  }
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
  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger += 1000;  // 1 per second
    battVolt = (1000 * analogRead(BATT_PIN)) / 451;
  } 
}


/**************************************************************************.
 *
 * compass()  Read the compass
 *
 ***************************************************************/
void myCompass() {

}


/**************************************************************************.
 * motorIdle() Power down if motor idle for period
 **************************************************************************/
void motorIdle() {
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
      isRunReady = isRunReady ? false : true;
    }
  }
}

/**************************************************************************.
 * sonar()
 **************************************************************************/
void sonar() {
  static unsigned long sonarTrigger = 0;
  if (timeMilliseconds > sonarTrigger) {
    sonarTrigger = timeMilliseconds + 60UL;
    sonarRight = ((float) analogRead(SONAR_RIGHT_AN)) * SONAR_SENS; // to feet
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
//
//boolean isBitClear(int test, byte b) {
//  return ((test & b) == 0);
//}
//
//boolean isBitSet(int test, byte b) {
//  return ((test & b) != 0);
//}
