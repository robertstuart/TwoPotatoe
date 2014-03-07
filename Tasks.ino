
const int MAX_BATT_COUNT = 100;  // Number of consecutive bad reading before action.
const float BATTERY_WARNING = 10.9f;  // about 10% capacity
const float BATTERY_CRITICAL = 10.0f; // about 1% cap
const int AUDIO_SEQ_MAX = 50;

// Timigs for tasks
const int BATTERY_INTERVAL = 500;  // milliseconds between battery reading
const int ERROR_INTERVAL = 1000;  // milliseconds between error reports

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

byte blinkPattern1[] = {1,2,2,2,2,2,2,2}; // red on, flash green
byte blinkPattern2[] = {2,1,1,1,1,1,1,1}; // green on, flash red
byte blinkPattern3[] = {2,2,1,1}; // rapid red-green
byte blinkPattern4[] = {3,3,3,3,0,0,0,0}; // both slow flash
byte* blinkPattern = blinkPattern1;
int blinkState = 99; // force change
int blinkPtr = 0;
int patternSize = 1;

const byte TASK_NONE = 0;
const byte TASK_PING = 1;
const byte TASK_BLINK = 2;
const byte TASK_BATT = 3;
const byte TASK_ERROR = 4;
const byte TASK_TONE = 5;
const byte TASK_RUNSTATE = 6;
const byte TASK_MODE = 7;
const byte TASK_VALSET = 8;

byte taskList[] = {0,TASK_TONE,0,TASK_RUNSTATE,0,TASK_BLINK,0,0,TASK_BATT,0,
                   0,TASK_TONE,0,TASK_MODE    ,0,TASK_BLINK,0,0,0,0,
                   0,TASK_TONE,0,TASK_RUNSTATE,0,TASK_BLINK,0,0,TASK_ERROR,0,
                   0,TASK_TONE,0,TASK_RUNSTATE,0,TASK_BLINK,0,0,0,0,
                   0,TASK_TONE,0,TASK_VALSET  ,0,TASK_BLINK,0,0,0,0};
                   

/***************************************************************
 *
 * tasks()
 *
 *       call misc. tasks periodically
 *       This routine is called in the main loop with the
 *       algorithm which is normally 100/sec.
 *       only do one before returning to main loop
 *
 ***************************************************************/
 void tasks() {
   safeAngle();
   gravity();
   
   taskPtr = ++taskPtr % sizeof(taskList);
   switch (taskList[taskPtr]) {
     case TASK_NONE:
       break;
     case TASK_BLINK:
       break;
     case TASK_BATT:
       battery();
       break;
     case TASK_ERROR:
       break;
     case TASK_TONE:
       blink();
       break;
     case TASK_RUNSTATE:
      cmdIntPC(CMD_RUN_STATE_STAT, runState);
       break;
     case TASK_MODE:
       cmdIntPC(CMD_MODE_STAT, mode);
       break;
     case TASK_VALSET:
       cmdIntPC(CMD_VAL_SET_STAT, valSetStat);
       break;
     default:
       break;
   }
 
//   taskMilliseconds = millis();
//   
//   safeAngle();
//   gravity();
//   
//   if (taskMilliseconds > batteryTrigger) {
//     batteryTrigger = taskMilliseconds + BATTERY_INTERVAL;
//     battery();
//     return;
//   }
//   
//   if (taskMilliseconds > blinkTrigger) {
//     blink();
//     return;
//   }
//   
//   if (taskMilliseconds > errorTrigger) {
//     errorTrigger = taskMilliseconds + ERROR_INTERVAL;
//     errorReport();
//     return;
//   }
 }



/*********************************************************
 *
 * safeAngle()
 *
 *     Check to see if we have fallen sidways or forwards.
 *     If so, turn off motors.
 *
 *********************************************************/
void safeAngle() {
  if ((abs(gaXAngle) > 45.0) || ((abs(gaYAngle) > 35))) {
    tipCount++;
    if (tipCount > 2) {
      upright = false;
    }
  }
  else {
    tipCount = 0;
    upright = true;
  }
}  // End upright().




/***************************************************************
 *
 * battery()
 *
 *       check battery, send out status, turn off if too low
 *
 ***************************************************************/
void battery() {

  // Send out the battery voltage
  int sensorValue = analogRead(BATTERY_PIN);
  batteryVoltage = sensorValue * BATT_ATOD_MULTIPLIER;
  cmdFloatPC(CMD_BATTVOLT_VAL, batteryVoltage);

  // Check for warning condition.
  if (batteryVoltage < BATTERY_WARNING) {
    warningCount++;
    if (warningCount > MAX_BATT_COUNT) {
//      beep(100, 1000);
      warningCount = 0;
    }
  } 
  else {
    warningCount = 0;
  }

  // Check for critical condition.
  if (batteryVoltage < BATTERY_CRITICAL) {
    criticalCount++;
    if (criticalCount > MAX_BATT_COUNT) {
      digitalWrite(PWR_PIN, LOW);  // Power down TwoPotatoe
    }
  } 
  else {
    criticalCount = 0;
  }
}



/***************************************************************
 *
 * errorReport()
 *
 *       Report the number of error of each type to the PC
 *
 ***************************************************************/
void errorReport() {
  if (byteCountErrorsHc) {
    debugInt("byteCountErrorsHc", byteCountErrorsHc);
    byteCountErrorsHc = 0;
  }
  if (pingHcTpErrors) {
    debugInt("pingHcTpErrors", pingHcTpErrors);
    pingHcTpErrors = 0;
  }
  if (pingPcTpErrors) {
    debugInt("pingPcTpErrors", pingPcTpErrors);
    pingPcTpErrors = 0;
  }
  if (unknownPcSingleCmdErrors) {
    debugInt("unknownPcSingleCmdErrors", unknownPcSingleCmdErrors);
    unknownPcSingleCmdErrors = 0;
  }
  if (unknownPcParamCmdErrors) {
    debugInt("unknownPcParamCmdErrors", unknownPcParamCmdErrors);
    unknownPcParamCmdErrors = 0;
  }
  if (unknownHcSingleCmdErrors) {
    debugInt("unknownHcSingleCmdErrors", unknownHcSingleCmdErrors);
    unknownHcSingleCmdErrors = 0;
  }
  if (unknownHcParamCmdErrors) {
    debugInt("unknownHcParamCmdErrors", unknownHcParamCmdErrors);
    unknownHcParamCmdErrors = 0;
  }
}



/***************************************************************
 *
 * blink()
 *
 *       blink the LEDs
 *
 ***************************************************************/
 void blink() {
   // Change the blinking pattern if the state has changed.
   if (runState != blinkState) {
     blinkState = runState;
     blinkPtr = 0;
     switch (runState) {
       case STATE_RESTING:
         blinkPattern = blinkPattern2;
         patternSize = sizeof(blinkPattern2);
         break;
       case STATE_READY:
         blinkPattern = blinkPattern3;
         patternSize = sizeof(blinkPattern3);
         break;
       case STATE_RUNNING:
         blinkPattern = blinkPattern1;
         patternSize = sizeof(blinkPattern1);
         break;
      default:
        break;
     }
   }
   
   // blink
   blinkPtr = ++blinkPtr % patternSize;
   byte blink = blinkPattern[blinkPtr];
   if (blink & 1) {
     digitalWrite(GREEN_LED_PIN, HIGH);
   }
   else {
     digitalWrite(GREEN_LED_PIN, LOW);
   }
   if (blink & 2) {
     digitalWrite(RED_LED_PIN, HIGH);
   }
   else {
     digitalWrite(RED_LED_PIN, LOW);
   }
 }


void gravity() {
    pressure = analogRead(PRESSURE_PIN);
    if (pressure > 300) {
      if (pressureCount > 2) {
        sitting = false;
      } 
      else {
        pressureCount++;
      }
    }
    else {
      pressureCount = 0;
      sitting = true;
    }
}

//
//void audio() {
//  if (audioSeqIndex < audioSeqCount)  {
//    tone(AUDIO_PIN, audioSeqPitch[audioSeqIndex]);
//    audioTrigger = taskMilliseconds + audioSeqDur[audioSeqIndex++];
//  }
//  else { // sequence complete
//    noTone(AUDIO_PIN);
//    audioTrigger = INT_MAX;
//  }
//}

//void siren() {
//  for (int x = 0; x < 5; x++) {
//    for (int y = 500; y < 1000; y++) {
//      tone(AUDIO_PIN, y);
//      delay(2);
//    }    
//  }
//
//  //  for (int x = 0; x < 5; x++) {
//  //    for (int i = 1000; i > 0 ; i -= 5) {
//  //      delayMicroseconds(500 + i);
//  //      analogWrite(DAC0, 50);
//  //      delayMicroseconds(500 + i);
//  //      analogWrite(DAC0, 0);
//  //    }
//  //  }
//}

//void audioStart(int pitch[], int dur[], size)
//
//
//// Sends out a single beep
//void beep(int pitch, int duration) {
//  beepPitch[0] = pitch;
//  beepDur[0] = duration;
//  audioStart(*beepPitch, *beepDur, 1);
//}

