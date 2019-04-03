/******************************************************************************
 *                               Run
 *****************************************************************************/
double accelFps = 0.0;
double coAccelFps = 0.0;
double lpfAccelFps = 0.0;
double lpfTpFps = 0.0;
float fpsCorrection = 0.0f;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;
float tpFps = 0.0;

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
//  beep(BEEP_UP);
//  startGyroDrift();
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
//  static double lpfAccelFpsOld = 0.0;
//  static double tp7OldSpeedError = 0.0;
  static double oldAccelFps = 0.0D;
 
  // Compute Center of Oscillation speed (cos)
  rotation3 = -gyroPitchDelta * valT;  // 4.5
  cos3 = wFps + rotation3;
  // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  lpfCos3 = (lpfCos3Old * valV) + (cos3 * (1.0D - valU));
  lpfCos3Accel = lpfCos3 - lpfCos3Old;
  lpfCos3Old = lpfCos3;

  // Compute the acceleration speed
//  double rad = (gaPitch + ZERO_ANGLE) * DEG_TO_RAD;
//  double yG = ((double) lsm6.a.y) / ONE_G_Y;
//  double zG = ((double) lsm6.a.z) / ONE_G_Z;
//  double yAccel = -(((cos(rad) * yG) + (sin(rad) * zG)) * ACCEL_TO_FPS);
//  accelFps += yAccel;

  // lp filter the accel value.
//  double lpfAccelFps = (lpfAccelFpsOld * (*currentValSet).a) + (accelFps * (1.0 - (*currentValSet).v)); // 0.9
//  double lpfAccelFpsDelta = lpfAccelFpsOld - lpfAccelFps;
//  lpfAccelFpsOld = lpfAccelFps;
  
  if (!isRunning) accelFps = lpfAccelFps = 0.0;

  // Complementary filter with COS. Try to minimizen jumping & loss of traction effects.
  // 0.98, High value places more emphasis on accel.
  double accelFpsDelta = accelFps - oldAccelFps;
  oldAccelFps = accelFps;
  coAccelFps += accelFpsDelta;
  coAccelFps = (coAccelFps * valW) + (lpfCos3 * (1.0 - valW));
//  lpfcoAccelFps -= lpfAccelFpsDelta;
//  lpfcoAccelFps = (lpfcoAccelFps * (*currentValSet).w) + (lpfCos3 * (1.0 - (*currentValSet).w));

  // Choose which of the computations to use.  Uncomment just one.
  tpFps = lpfCos3;
//  float tpFps = coAccelFps;
//  float tpFps = lpfcoAccelFps;

  tp7ControllerSpeed = controllerY * SPEED_MULTIPLIER; 

  // Find the speed error.  Constrain rate of change.
  double tp7SpeedError = tp7ControllerSpeed - tpFps;
//  double speedDiff = tp7SpeedError - tp7OldSpeedError;
//  if (speedDiff > 0.0D) tp7OldSpeedError += FPS_ACCEL;
//  else tp7OldSpeedError -= FPS_ACCEL;
//  tp7SpeedError = tp7OldSpeedError;

  // compute a weighted angle to eventually correct the speed error
  targetAngle = -(tp7SpeedError * valX); //** 4.0 ******** Speed error to angle *******************
  
//targetAngle = 0.0;  
//fps = 0.0;
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetAngle = constrain(targetAngle, -20.0, 20.0);

  // Compute angle error and weight factor
  angleError = targetAngle - gaPitch;
  fpsCorrection = angleError * valY; // 0.4 ******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target wheel speed.
  targetWFps = fpsCorrection + tpFps;
//  targetWFps = fpsCorrection;

  // These routines set the steering values.
//  if (isRouteInProgress) steerRoute(targetWFps);
  steer(targetWFps);

} // end aTp7() 



/***********************************************************************.
 *  sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;
  
//  if (!(logLoop % 125)) log2PerSec();
  if (!(logLoop % 25)) log10PerSec();
//  routeLog(); //  250/sec
//  if (!(logLoop % 12)) log20PerSec(); // 20/sec  
//  if (!(logLoop % 2)) log104PerSec(); // 125/sec  
//  log208PerSec();
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
  sprintf(message, "wFpsRight %4.2f   wFpsLeft: %4.2f", wFpsRight, wFpsLeft);
//  sprintf(message, "aPitch %4.2f   gPitch: %4.2f   gaPitch: %4.2f", aPitch, gPitch, gaPitch);
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
    sendUpMsg(TOUP_FPS, 2, tpFps);
    sendUpMsg(TOUP_DIST, 2, ((float) tickPosition) / TICKS_PER_FOOT);
  }
}


/***********************************************************************.
 *  steer() 
 ***********************************************************************/
void steer(float fp) {
  float speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX; 
  targetWFpsRight = fp - speedAdjustment;
  targetWFpsLeft = fp + speedAdjustment;
}
