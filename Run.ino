/******************************************************************************
 *                               Run.ino
 *****************************************************************************/
float fpsCorrection = 0.0f;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;
float lpfFps = 0.0;
float lpfFpsOld = 0.0;
float fpsError = 0;
float controllerSpeed = 0.0;
bool isDownTrigger = false;

/*****************************************************************************-
 *   runLoop() Main loop
 *****************************************************************************/
void runLoop() {
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  yePattern = BLINK_OFF;
  buPattern = BLINK_OFF;
  rePattern = BLINK_OFF;
  gnPattern = BLINK_OFF;
  sendWaMsg(SEND_BEEP, T_UP2);
  while (mode == MODE_RUN) { // main loop
    commonTasks();
    if (isNewImu()) {
      checkMotors();
      if (isGettingUp) gettingUp();
      else if (isGettingDown) gettingDown();
      else if (isRunningOnGround) runOnGround();
      else if (isBowlMode) runBowl();
      else run();
      sendLog();
      sendUpData();
      sendWaMsg(SEND_WATCH, 0);  // Update watchdog.
      //sprintf(message, "%d  %.2f  %5.2f", 0, controllerSpeed, routeSteer);
      //Serial.println(message);
    }
  }
}



const double ACCEL_TO_FPS = 0.06;
/*****************************************************************************-
 *   run() Balance and turn.
 *****************************************************************************/
/*****************************************************************************-
 *  Include explanation of balancing. Use Segway/hoverboard as one end of a 
 *  continuum and a tall inverted pendulum bot as the other.  Modify this 
 *  algorith to directly reflect this by having one variable be the height
 *  to the CO.  Also include instrutions for tuning.
 *****************************************************************************/
 
void run() {
  //
  // // Compute Center of Oscillation fps
  // pitchFps = gyroPitchDelta * valT;  // 1.5 Speed of the cos due to change in pitch (rotation)
  // fps = wFps - pitchFps;
  //
  // // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  // lpfFps = (lpfFpsOld * valU) + (fps * (1.0D - valU));
  // lpfFpsOld = lpfFps;

  if (isRouteInProgress) {
    controllerSpeed = routeFps;
  } else {
    controllerSpeed = getControllerSpeed();
  }
  // Find the fps error.  Constrain rate of change.
  fpsError = controllerSpeed - getCoFps();

  // compute a weighted angle to eventually correct the fps error
  targetAngle = -(fpsError * valX); //** 4.0 ******** Speed error to angle *******************

  // Compute maximum angles for the current wheel fps and enforce limits.
  targetAngle = constrain(targetAngle, -35.0, 35.0);

  // Compute angle error and weight factor
  angleError = targetAngle - getTpPitch();

  fpsCorrection = angleError * valY; // 0.4 ******************* Angle error to fps *******************
  fpsCorrection += gyroPitchDelta * -0.5;  // add "D" to reduce overshoot

  // Add the angle error to the base fps to get the target wheel fps.
  targetWFps = fpsCorrection + lpfFps;

  // These routines set the steering values.
  steer(targetWFps);
} // end run()



/*****************************************************************************-
 *    runBowl() Run in a bowl
 *****************************************************************************/
void runBowl() {

  float camSlope = getCamSlope();
  if (isIllegalCam(camSlope)) {
    run();
  } else {
    float targetAngle = gPitch;
    fpsCorrection = angleError * valY; // 0.4 ******************* Angle error to fps *******************
    fpsCorrection += gyroPitchDelta * -0.5;  // add "D" to reduce overshoot

    // Add the angle error to the base fps to get the target wheel fps.
    targetWFps = fpsCorrection + getCoFps();

    // These routines set the steering values.
    steer(targetWFps);
  }
} // end run()



/***********************************************************************.
 *  runOnGround()
 *****************************************************************************/
void runOnGround() {
  steer(getControllerSpeed());
}



/***********************************************************************.
 *  getTpPitch()  return the pitch from maPitch or xxxx
 ***********************************************************************/
float getTpPitch() {
  float pitch;
  if (isCamPitch && isUpRunning) pitch = camPitch;
  else pitch = (maPitch - valZ);
  return pitch;
}



/***********************************************************************.
 *  sendLog() Called 200 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;

//    logUp();
//    if (!(logLoop % 100)) log2PerSec();
//    if (!(logLoop % 20)) log10PerSec();
//    routeLog(); //  200/sec
//    if (!(logLoop % 10)) log20PerSec(); // 20/sec
//    if (!(logLoop % 2)) log104PerSec(); // 125/sec
//    log200PerSec();
}



/*****************************************************************************-
 *    getCoFps() Return the fps of the "robot".  This is the speed of The
 *             computed center of oscillation (Co).
 *****************************************************************************/
float getCoFps() {
  static float lpfFpsOld = 0.0;

  // Compute Center of Oscillation fps
  float pitchFps = gyroPitchDelta * valT;  // 1.5 Speed of the cos due to change in pitch (rotation)
  float fps = wFps - pitchFps;

  // Low pass filter to resolve time difference between gyro and ticks
  // 0.92 .u value: 0.0 = no lp filtering, large values give slow response
  lpfFps = (lpfFpsOld * valU) + (fps * (1.0D - valU));
  lpfFpsOld = lpfFps;
  return lpfFps;
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
//    sprintf(message, "gyroPitchRaw: %7d   timeDriftPitch: %7.2f   gyroPitchDelta: %7.2f  gPitch: %7.2f", gyroPitchRaw, timeDriftPitch, gyroPitchDelta * 100.0, gPitch);
//    sprintf(message, "maPitch %6.2f  caPitchPitch: %6.2f", maPitch, caPitch);
    sprintf(message, "tickFeet: %6.2f", feetPosition);
//    dPrint("aPitch: ", aPitch, 2); dPrintln("    gaPitch: ", gaPitch, 2);
//    sprintf(message, "gPitch %7.2f   gRoll: %7.2f   gYaw: %7.2f   aPitch %7.2f   aRoll: %7.2f", gPitch, gRoll, gYaw, aPitch, aRoll);
//      sprintf(message, "x: %5d     t: %5d     z: %5d", (int) lsm6.a.x, (int) lsm6.a.y, (int) lsm6.a.z);
//    sprintf(message, "gVert: %5.2f   gHoriz: %5.2f   speedGHoriz: %7.2f   distGHoriz: %7.2f", gVert, gHoriz, speedGHoriz, distGHoriz);
    Serial.println(message);
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
    sendUpData() Send data every cycle.
 ***********************************************************************/
void sendUpData() {
  sendUpMsg(TOUP_HEADING, 2, -maYaw);
  sendUpMsg(TOUP_FPS, 2, lpfFps);
  sendUpMsg(TOUP_PITCH, 2, maPitch);
  sendUpMsg(TOUP_V_ACCEL, 2, vAccel);
  sendUpMsg(TOUP_TICKS, tickPosition); // ticks is always last
}



/***********************************************************************.
 *  getControllerSpeed()  Return the controller speed depending upon
 *                        the current "tuning" from the HC.
 ***********************************************************************/
float getControllerSpeed() {
  float mult = 20.0;
  if (speedTuning == 0) mult = 6.0;
  else if (speedTuning == 1) mult = 12.0;
  return(controllerY * mult);
}



/***********************************************************************.
 *  steer()
 ***********************************************************************/
void steer(float fp) {
  float fpsAdjustment = 0.0;
  if (isRouteInProgress) {
    fpsAdjustment = (((1.0 - abs(routeSteer)) * 1.5) + 0.5) * routeSteer;
  } else {
    fpsAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX;
  }
  targetWFpsRight = fp - fpsAdjustment;
  targetWFpsLeft = fp + fpsAdjustment;
  //sprintf(message, " %5.2f     %5.2f   %2d", fp, routeSteer, isRouteInProgress);
  //Serial.println(message);
}



/******************************************************************************
    setGetUp()
 *****************************************************************************/
void setGetUp() {
  if (!isUpright) {
    isRunReady = true;
    isGettingUp = true;
    gettingUpStartTime = timeMilliseconds;
    //sendUpMsg(TOUP_START_LOG, true);
  }
  Serial.println("UP!");
}



/******************************************************************************
    gettingUp()
 *****************************************************************************/
void gettingUp() {
  const int REV_TIME = 1200;
  float tFps = 0.0;
  unsigned long runTime = timeMilliseconds - gettingUpStartTime;
  if (runTime > 1800) {  // Give up if not up yet.
    //sendUpMsg(TOUP_START_LOG, false);
    isGettingUp = false;
    isRunReady = false;
    return;
  }
  float ab = abs(gaPitch);
  if (ab < 15.0) {
    isGettingUp = false;
    isRunReady = true;
    return;
  }
  bool isBack = (gaPitch > 0.0) ? true : false;
  if (runTime < REV_TIME) {  // Go in reverse at start.
    tFps = -((runTime * 0.004) + 0.1);
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
    setGetDown()
 *****************************************************************************/
void setGetDown() {
  if (isUpright && isRunning) {
    gettingDownStartTime = timeMilliseconds;
    isGettingDown = true;
    isDownTrigger = false;
  }
}



/******************************************************************************
    gettingDown()
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
  if (isDownTrigger == false) {
    if      (gaPitch < 10.0) tFps = 1.5;
    else if (gaPitch < 50.0) tFps = 0.0;
    else   {
      isDownTrigger = true;
      pulseStartTime = timeMilliseconds;
    }
  } else {
    unsigned long pulseTime = timeMilliseconds - pulseStartTime;
    if (pulseTime < 150) {
      tFps = -7.0;
    } else {
      isGettingDown = false;
      isRunReady = false;
      return;
    }
  }
  targetWFpsRight = targetWFpsLeft = tFps;
}
