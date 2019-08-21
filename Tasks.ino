/*****************************************************************************-
 *                        Tasks
 *****************************************************************************/

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
  timeMilliseconds = millis();
  readWa();
  readXBee();  // Read commands from PC or Hand Controller
//  readUp(); 
  setRunningState();
  setLedStates();
//  gyroTemperature();
//  testUp(); 
  blink13();
}
void testUp() {
  static  unsigned  int testUpTrigger = 0;
  if (timeMilliseconds > testUpTrigger) {
    testUpTrigger = timeMilliseconds + 500;
    sprintf(message, "Test from Teensy: %d", testUpTrigger);
    sendUpMsg(TOUP_LOG, message);  
  }
}


/*****************************************************************************-
 * setRunningState()
 *****************************************************************************/
void setRunningState() {
  // Set the runnng bit to control motors
  isRunning = (isRunReady && (isUpright || isGettingUp || isGettingDown)) ? true : false;
  if (isRunning) yePattern = BLINK_ON;
  else if (isRunReady) yePattern = BLINK_FF;
  else yePattern = BLINK_OFF;
}



/*****************************************************************************-
 * setLedStates()  Send an led message every time the pattern variable changes.
 *****************************************************************************/
void setLedStates() {
  static int waYePattern = BLINK_ON;
  static int waBuPattern = BLINK_ON;
  static int waRePattern = BLINK_ON;
  static int waGnPattern = BLINK_ON;
  
  if (yePattern != waYePattern ) {
    sendWaMsg(SEND_BLINK, LED_SW_YE, yePattern); 
    waYePattern = yePattern;
  }
  if (buPattern != waBuPattern ) {
    sendWaMsg(SEND_BLINK, LED_SW_BU, buPattern); 
    waBuPattern = buPattern;
  }
  if (rePattern != waRePattern ) {
    sendWaMsg(SEND_BLINK, LED_SW_RE, rePattern); 
    waRePattern = rePattern;
  }
  if (gnPattern != waGnPattern ) {
    sendWaMsg(SEND_BLINK, LED_SW_GN, gnPattern); 
    waGnPattern = gnPattern;
  }
}



/*****************************************************************************-
 * checkUpright() Check to see if we have fallen sidways or forwards.
 *****************************************************************************/
void checkUpright() {
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

//  boolean cState = ((abs(mqPitch) < 45.0) && ((abs(mqRoll) < 45.0))); // Current real state
  boolean cState = (abs(gaPitch) < 40.0); // Current real state
  if (!cState && tState) {
    tTime = timeMilliseconds; // Start the timer for a state change to fallen.
  } else if (!cState) {
    if ((timeMilliseconds - tTime) > 50) {
      isUpright = false; 
    }
  } else {
    isUpright = true;
  }
  tState = cState;
}



/*****************************************************************************-
 * blink13()  Just blink the on-board led
 *****************************************************************************/
void blink13() {
  static  unsigned  int blinkTrigger = 0;
  static  unsigned  int blinkPat = 0;
  boolean b = false;
  if (timeMilliseconds > blinkTrigger) {
    blinkTrigger = timeMilliseconds + 100;
    blinkPat++;
    blinkPat = blinkPat % 6;  
    if (blinkPat == 1)  b = true; 
    if (blinkPat == 3)  b = true; 
    digitalWrite(LED13_PIN, b ? HIGH : LOW);
  }
}



/*****************************************************************************-
 *  rangeAngle() Set angle value between -180 and +180
 *****************************************************************************/
float rangeAngle(float angle) {
  int loops = (int) (angle / 360.0);
  float x = angle - (((float) loops) * 360.0);
  if (x <= -180.0) x += 360.0;
  if (x > 180.0) x -= 360.0;
  return x;
}
