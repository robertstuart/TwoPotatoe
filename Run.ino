/******************************************************************************
 *                               Run
 *****************************************************************************/
float fpsCorrection = 0.0f;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;
float lpfFps = 0.0;
float lpfFpsOld = 0.0;
float fpsError = 0;
float fps = 0.0;
float pitchFps = 0.0;
float controllerSpeed = 0.0;
bool isDownTrigger = false;

/*****************************************************************************-
 *  runLoop() Run the TwoPotatoe algorithm.
 *             This loops while in the RUN mode.
 *****************************************************************************/
void runLoop() {
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  setHeading(0.0D);
  yePattern = BLINK_OFF;
  buPattern = BLINK_OFF;
  rePattern = BLINK_OFF;
  gnPattern = BLINK_OFF;
  sendWaMsg(SEND_BEEP, T_UP2);
  resetTicks();
  while(mode == MODE_RUN) { // main loop
    commonTasks();
    if (isNewImu()) {
      checkMotors();
//      setNavigation();
      if (isGettingUp) gettingUp();
      else if (isGettingDown) gettingDown();
      else run(); 
      sendLog();
      checkUpright();
//      sendUpData();
      sendWaMsg(SEND_WATCH, 0);  // Update watchdog.
    } 
  }
}



const double ACCEL_TO_FPS = 0.06;
/*****************************************************************************-
 *  run() Balance and turn.
 *****************************************************************************/
void run() {
  if (isLogging) {
    sprintf(message, "%.2f,%.2f", gaPitch, maPitch);
    sendUpMsg(TOUP_LOG, message);
  }
 
  // Compute Center of Oscillation fps 
  pitchFps = gyroPitchDelta * valT;  // 4.5 Speed of the cos due to change in pitch (rotation)
  fps = wFps + pitchFps;
    
  // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  lpfFps = (lpfFpsOld * valU) + (fps * (1.0D - valU));
  lpfFpsOld = lpfFps;

  controllerSpeed = controllerY * SPEED_MULTIPLIER; 

  // Find the fps error.  Constrain rate of change.
  fpsError = controllerSpeed - lpfFps;

  // compute a weighted angle to eventually correct the fps error
  targetAngle = -(fpsError * valX); //** 4.0 ******** Speed error to angle *******************
  
  // Compute maximum angles for the current wheel fps and enforce limits.
  targetAngle = constrain(targetAngle, -35.0, 35.0);

  // Compute angle error and weight factor
  angleError = targetAngle - maPitch;
//  angleError = targetAngle - gaPitch;
  fpsCorrection = angleError * valY; // 0.4 ******************* Angle error to fps *******************

  // Add the angle error to the base fps to get the target wheel fps.
  targetWFps = fpsCorrection + lpfFps;

  // These routines set the steering values.
  steer(targetWFps);
//    sprintf(message, "%5.2f %5.2f %5.2f",  mqPitch, pitchFps, targetWFps);
//    Serial.println(message);

} // end aTp7() 



/***********************************************************************.
 *  sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;

//  logUp();
//  if (!(logLoop % 125)) log2PerSec();
//  if (!(logLoop % 25)) log10PerSec();
//  routeLog(); //  250/sec
//  if (!(logLoop % 12)) log20PerSec(); // 20/sec  
//  if (!(logLoop % 2)) log104PerSec(); // 125/sec  
//  log208PerSec();
}

void logUp() {
//  sprintf(message, "%5.2f,%5.2f,%5.2f,%5.2f", controllerSpeed, aPitch, gPitch ,wFps);
//  sendUpMsg(TOUP_LOG, message);
}
void log2PerSec() {
//  Serial.print(isRunning); Serial.print(isRouteInProgress); Serial.println(isRunReady);
//  sprintf(message, "gyroHeading %4.2f \t aPitch: %4.2f \t gaPitch: %4.2f", gyroHeading, aPitch, gaPitch);
//  sendBMsg(SEND_MESSAGE, message);
//  Serial.print(forceLeft);
//  Serial.print(tab);
//  Serial.print(forceRight);
//  Serial.println();
}

void log10PerSec() {
//  sprintf(message, "gyroPitchRaw: %7d   timeDriftPitch: %7.2f   gyroPitchDelta: %7.2f  gPitch: %7.2f", gyroPitchRaw, timeDriftPitch, gyroPitchDelta * 100.0, gPitch);
//  sprintf(message, "aPitch %4.2f   gPitch: %4.2f   gaPitch: %4.2f", aPitch, gPitch, gaPitch);
//  dPrint("aPitch: ", aPitch, 2); dPrintln("    gaPitch: ", gaPitch, 2);
//  sprintf(message, "gPitch %7.2f   gRoll: %7.2f   gYaw: %7.2f   aPitch %7.2f   aRoll: %7.2f", gPitch, gRoll, gYaw, aPitch, aRoll);
//    sprintf(message, "x: %5d     t: %5d     z: %5d", (int) lsm6.a.x, (int) lsm6.a.y, (int) lsm6.a.z);
//  sprintf(message, "gVert: %5.2f   gHoriz: %5.2f   speedGHoriz: %7.2f   distGHoriz: %7.2f", gVert, gHoriz, speedGHoriz, distGHoriz);
//  Serial.println(message);
}

void log20PerSec() {
}

void log104PerSec() {
}

void log208PerSec() {
//  sprintf(message, "%.4f,%.2f,%.2f", gHoriz, gX, gZ, );
//  sendUpMsg(TOUP_LOG, message);
//  Serial.println(message);
}
     


/***********************************************************************.
 *  sendUpData() Send periodic loc, heading, & pitch to Up.
 ***********************************************************************/
void sendUpData() {
  static int sendCount = 0;
  if ((++sendCount % 4) == 0) {
    sendUpMsg(TOUP_X, 2, currentLoc.x);
    sendUpMsg(TOUP_Y, 2, currentLoc.y);
//    sendUpMsg(TOUP_HEADING, 2, gcHeading);
//    sendUpMsg(TOUP_PITCH, 0, gaPitch);
    sendUpMsg(TOUP_FPS, 2, lpfFps);
    sendUpMsg(TOUP_DIST, 2, ((float) tickPosition) / TICKS_PER_FOOT);
  }
}


/***********************************************************************.
 *  steer() 
 ***********************************************************************/
void steer(float fp) {
  float fpsAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX; 
  targetWFpsRight = fp - fpsAdjustment;
  targetWFpsLeft = fp + fpsAdjustment;
}



/******************************************************************************
 *  setGetUp()
 *****************************************************************************/
void setGetUp() {
  if(!isUpright) {
    isRunReady = true;
    isGettingUp = true;
    gettingUpStartTime = timeMilliseconds;
sendUpMsg(TOUP_START_LOG, true);
  }
  Serial.println("UP!");
}



/******************************************************************************
 *  gettingUp()
 *****************************************************************************/
void gettingUp() {
  const int REV_TIME = 500;
  float tFps = 0.0;
  unsigned long runTime = timeMilliseconds - gettingUpStartTime;
  if (runTime > 1000) {  // Give up if not up yet.
sendUpMsg(TOUP_START_LOG, false);
    isGettingUp = false;
    isRunReady = false;
    return;
  }
  float ab = abs(gaPitch);
  if (ab < 15.0) {
    isGettingUp = false;
    isRunReady = true;
sendUpMsg(TOUP_START_LOG, false);
    return;
  }
  bool isBack = (gaPitch > 0.0) ? true : false;
  if (runTime < REV_TIME) {  // Go in reverse at start.
    tFps = -((runTime * 0.02) + 0.5);
  } else { 
//    tFps = 0.0;
//    tFps = 5.0;
//    tFps = (((runTime - REV_TIME) * 0.05) + 0.5);
    if      (ab > 80.0) tFps = 5.0;
    else if (ab > 70.0) tFps = 7.0;
    else if (ab > 60.0) tFps = 10.0;
    else if (ab > 50.0) tFps = 10.0;
    else if (ab > 40.0) tFps = 8.0;
    else if (ab > 30.0) tFps = 5.0;
    else if (ab > 25.0) tFps = 4.0;
    else if (ab > 20.0) tFps = 3.0;
    else if (ab > 15.0) tFps = 2.0;
    else tFps = 2.0;
  }
  if (isBack) tFps = -tFps;
  targetWFpsRight = targetWFpsLeft = tFps;
//sprintf(message, "%d,%.2f,%.4f,%.1f,%.1f", runTime, gaPitch, gyroPitchDelta, tFps, wFps);
//sendUpMsg(TOUP_LOG, message);
}



/******************************************************************************
 *  setGetDown()
 *****************************************************************************/
void setGetDown() {
  if (isUpright && isRunning) {
    gettingDownStartTime = timeMilliseconds;
    isGettingDown = true;
    isDownTrigger = false;
  }
}



/******************************************************************************
 *  gettingDown()
 *****************************************************************************/
void gettingDown() {
  static unsigned long pulseStartTime  = 0UL;
  unsigned long runTime = timeMilliseconds - gettingDownStartTime;
  float tFps = 0.0;
  if (runTime > 2000) { 
    isGettingDown = false;
    isRunReady = false;
    return;
  }
  if (isDownTrigger == true) {
    unsigned long pulseTime = timeMilliseconds - pulseStartTime;
    if (pulseTime < 200) {
      tFps = -10.0;
    } else {
      isGettingDown = false;
      isRunReady = false;
      return;
    }
  } else {
    if      (gaPitch < 10.0) tFps = 1.5;
    else if (gaPitch < 40.0) tFps = 0.0;
    else if (gaPitch < 60.0) tFps = 0.0;
    else   {
      isDownTrigger = true;
      pulseStartTime = timeMilliseconds;
    }
  }
  targetWFpsRight = targetWFpsLeft = tFps;
}
