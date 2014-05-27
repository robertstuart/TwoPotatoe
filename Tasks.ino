#include <DueTimer.h>

const int BATTERY_WARNING = 1090;  // about 10% capacity (centivolts)
const int BATTERY_CRITICAL = 1000; // about 1% cap (centivolts)

int beepCycleCount = 0;
boolean beepStat = false;
int *beepSequence;
int beepPtr = 0;

unsigned long batteryTimer = 0;
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


/*********************************************************
 *
 * setTp4RunningState()
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
void setTp4RunningState() {
  byte* blinkState = BLINK_SBYG;

  // Set the bit
  if (     ((tpState & TP_STATE_RUN_READY) != 0)
    && ((tpState & TP_STATE_UPRIGHT) != 0)
    && ((tpState & TP_STATE_ON_GROUND) != 0)) { 
    if (!(tpState & TP_STATE_RUNNING)) { // only zero for state change      
      tpState = tpState | TP_STATE_RUNNING;
      if (!isRouteInProgress) {
        targetHeading = tickHeading;
      }
    }
  }
  else {
    tpState = tpState & ~TP_STATE_RUNNING;
  }

  // set x, y, and blink state
  if (!(tpState & (TP_STATE_HC_ACTIVE | TP_STATE_PC_ACTIVE))) {
//    controllerY = 0.0f;
//    controllerX = 0.0f;
    blinkState = BLINK_SBYG;  // Slow blinking if no connection
  }
  else if ((tpState & TP_STATE_RUNNING) != 0) {
    blinkState = BLINK_F_RG; // Blue, flash Red if running.
  }
  else if ((tpState & TP_STATE_RUN_READY) > 0) {
    blinkState = BLINK_FRG; // Flash red-blue if ready to go.
  }
  else { // idle (none of the above).
    if (mode == MODE_TP5) {
      blinkState = BLINK_R_FB; 
    }
    else {
      blinkState = BLINK_B_FR;
    }
  }

  if (blinkState != blinkPattern) {
    setBlink(blinkState);
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
  if ((abs(gaPitchAngle) > 45.0) || ((abs(gaRollAngle) > 35))) {  // Not at a safe angle?
    if (timeMilliseconds > (uprightTime + 50)) { // more that 1/20th of a second?
      tpState = tpState & ~TP_STATE_UPRIGHT;
    }
  }
  else {
    tpState = tpState | TP_STATE_UPRIGHT;
    uprightTime = timeMilliseconds;
  }
}  // End upright().


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
  byte hcBit =  ((tHc + 500) > timeMilliseconds) ? TP_STATE_HC_ACTIVE : 0;
  byte pcBit =  ((tPc + 500) > timeMilliseconds) ? TP_STATE_PC_ACTIVE : 0;
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

  // Send out the battery voltage
  int sensorValue = analogRead(BATTERY_PIN);
  batteryVolt = 100 * (((float) sensorValue) * BATT_ATOD_MULTIPLIER);
  
  // Check for warning condition.
  if (batteryVolt < BATTERY_WARNING) {
    // Continuously low for a second?
    if ((batteryLastGood + 1000) < timeMilliseconds) {
      if (timeMilliseconds > warningTrigger) {
        beep(BEEP_WARBLE);
        warningTrigger += 60000;
      }
    }
  } 
  else {
    batteryLastGood = timeMilliseconds;
  }

  // Check for critical condition.
  if (batteryVolt < BATTERY_CRITICAL) {
    if ((timeMilliseconds + 10000) > warningTrigger) {
      digitalWrite(PWR_PIN, LOW);  // Power down TwoPotatoe
    }
  } 
}

// Set the blink pattern
void setBlink(byte* pattern) {
  blinkPattern = pattern;
  blinkPtr = 0;
}



/***************************************************************
 *
 * compass()  Read the compass
 *
 ***************************************************************/
 void myCompass() {
   
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
    int halfCycle = (1000000 /2) / freq;
    int dur = beepSequence[beepPtr + 1];
    beepCycleCount = (dur * 1000)/halfCycle;
    Timer3.start(halfCycle);
    beepPtr +=2;
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

