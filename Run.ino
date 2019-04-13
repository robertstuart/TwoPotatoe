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

/*****************************************************************************-
 *  runLoop() Run the TwoPotatoe algorithm.
 *             This loops while in the RUN mode.
 *****************************************************************************/
void runLoop() {
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  delay(200); // For switches?
  setHeading(0.0D);
  resetTicks();
  while(mode == MODE_RUN) { // main loop
    commonTasks();
    if (isNewImu()) {
      checkMotors();
//      setNavigation();
      run(); 
      doGyroDrift();
      sendLog();
      checkUpright();
//      sendUpData();
    } 
  }
}



const double ACCEL_TO_FPS = 0.06;
/*****************************************************************************-
 *  run() Balance and turn.
 *****************************************************************************/
void run() {
 
  // Compute Center of Oscillation fps 
  pitchFps = -gyroPitchDelta * valT;  // 4.5 Speed of the cos due to change in pitch (rotation)
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
  angleError = targetAngle - gaPitch;
  fpsCorrection = angleError * valY; // 0.4 ******************* Angle error to fps *******************

  // Add the angle error to the base fps to get the target wheel fps.
  targetWFps = fpsCorrection + lpfFps;

  // These routines set the steering values.
  steer(targetWFps);

} // end aTp7() 



/***********************************************************************.
 *  sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;

//  logUp();
//  if (!(logLoop % 125)) log2PerSec();
  if (!(logLoop % 25)) log10PerSec();
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
  sprintf(message, "gyroPitchRaw: %7d   timeDriftPitch: %7.2f   gyroPitchDelta: %7.2f  gPitch: %7.2f", gyroPitchRaw, timeDriftPitch, gyroPitchDelta * 100.0, gPitch);
//  sprintf(message, "aPitch %4.2f   gPitch: %4.2f   gaPitch: %4.2f", aPitch, gPitch, gaPitch);
//  dPrint("aPitch: ", aPitch, 2); dPrintln("    gaPitch: ", gaPitch, 2);
//  sprintf(message, "gPitch %4.2f   gRoll: %4.2f   gYaw: %4.2f", gPitch, gRoll, gYaw);
//    sprintf(message, "gPitch: %5.2f     gRoll: %5.2f     gYaw: %5.2f\n", gPitch, gRoll, gYaw);
  Serial.println(message);
}

void log20PerSec() {
//  sprintf(message,  "%5.2f\t%5.2f\t%5.2f\t", sonarLeft, sonarFront, sonarRight);
//  sendBMsg(SEND_MESSAGE, message); 
}

void log104PerSec() {
}


void log208PerSec() {
}
        


/***********************************************************************.
 *  sendUpData() Send periodic loc, heading, & pitch to Up.
 ***********************************************************************/
void sendUpData() {
  static int sendCount = 0;
  if ((++sendCount % 4) == 0) {
    sendUpMsg(TOUP_X, 2, currentLoc.x);
    sendUpMsg(TOUP_Y, 2, currentLoc.y);
    sendUpMsg(TOUP_HEADING, 2, gcHeading);
    sendUpMsg(TOUP_PITCH, 0, gaPitch);
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
