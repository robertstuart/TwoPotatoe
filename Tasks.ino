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
//  readXBee();  // Read commands from PC or Hand Controller
//  readUp(); 
//  controllerConnected();
  setRunningState();
//  gyroTemperature(); 
  blink13();
}



/*****************************************************************************-
 * setRunningState()
 *****************************************************************************/
void setRunningState() {
  static boolean oldIsUpright = false;
  static boolean oldIsRunReady = false;

  if ((oldIsUpright == isUpright) &&
     (oldIsRunReady == isRunReady)) return;  // Nothing to do.

  if (mode == MODE_RUN) {
    // Set the runnng bit to control motors
    isRunning = (isRunReady && isUpright) ? true : false;
  } else { // For all test modes, just set accoding to ready bit
    isRunning = isRunReady;
  }

  oldIsUpright = isUpright;
  oldIsRunReady = isRunReady;
  
  //  yellow led
  if (isRunning) sendWaMsg(SEND_BLINK, LED_SW_YE, BLINK_ON);
  else if (!isRunReady && isUpright) sendWaMsg(SEND_BLINK, LED_SW_YE, BLINK_FF);
  else if (isRunReady && !isUpright) sendWaMsg(SEND_BLINK, LED_SW_YE, BLINK_SB);
  else sendWaMsg(SEND_BLINK, LED_SW_YE, BLINK_SF); 
}


/*****************************************************************************-
 * checkUpright() Check to see if we have fallen sidways or forwards.
 *****************************************************************************/
void checkUpright() {
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

  boolean cState = ((abs(gaPitch) < 45.0) && ((abs(gaRoll) < 45.0))); // Current real state
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
 * controllerConnected()
 *****************************************************************************/
void controllerConnected() {
  isHcActive  =  ((tHc + 1000) > timeMilliseconds) ? true : false;
//  isPcActive =  ((tPc + 1000) > timeMilliseconds) ? true : false;
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



/*****************************************************************************-
 * dprint???()  Convenience functions for debug printouts.
 *****************************************************************************/
void dPrint(String s, int i) {
  Serial.print(s);
  Serial.print(i);
}
void dPrintln(String s, int i) {
  Serial.print(s);
  Serial.println(i);
}
void dPrint(String s, float f, int precision) {
  Serial.print(s);
  Serial.print(f, precision);
}
void dPrintln(String s, float f, int precision) {
  Serial.print(s);
  Serial.println(f, precision);
}
void dPrint(String s1, char* s2) {
  Serial.print(s1);
  Serial.print("\"");
  Serial.print(s2);
  Serial.print("\"");
}
void dPrintln(String s1, char* s2) {
  Serial.print(s1);
  Serial.print("\"");
  Serial.print(s2);
  Serial.println("\"");
}


//
//boolean isBitClear(int test, byte b) {
//  return ((test & b) == 0);
//}
//
//boolean isBitSet(int test, byte b) {
//  return ((test & b) != 0);
//}
