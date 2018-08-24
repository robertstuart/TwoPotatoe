/***********************************************************************.
 * ----- TP7 ------
 ***********************************************************************/
double accelFps = 0.0;
double coAccelFps = 0.0;
double lpfAccelFps = 0.0;
double lpfTpFps = 0.0;
float fpsCorrection = 0.0f;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;


/***********************************************************************.
 *  aTp7Run() 
 ***********************************************************************/
void aTp7Run() {
  unsigned long loopTrigger = micros();
  unsigned long debugTrigger = loopTrigger; //*****************
  boolean isGyroRead = false;
  boolean isAccRead = false;
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  currentValSet = &tp7;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200); // For switches?
  setHeading(0.0D);
  resetTicks();
  beep(BEEP_UP);
//  startGyroDrift();
  while(mode == MODE_2P) { // main loop
    commonTasks();
    if (timeMicroseconds > loopTrigger) {
      loopTrigger += 1000;
      checkMotors();
      if (isNewGyro()) {
        setGyroData();
        isGyroRead = true;
      }
      if (isNewAccel()) {
        setAccelData();
        isAccRead = true;
      }
      if (isGyroRead && isAccRead) {
       isGyroRead = isAccRead = false;
        setNavigation();
        aTp7(); 
        doGyroDrift();
        sendLog();
        safeAngle();
        sendUpData();
      }
    } 
  }
}



const int Y_BIAS = 0;
const double Y_ONE_G = -16500.0;
const double ACCEL_TO_SPEED = 0.000009;
const double COMP_TC = 0.98;
const double ACCEL_TC = 0.90;
//const float COMP_TC = 0.0; // Cos only
const double FPS_ACCEL = 0.05;
const double ZERO_ANGLE = 0.5;
const double ONE_G_Y = -16530.0D;
const double ONE_G_Z = 17029.0D;
const double ACCEL_TO_FPS = 0.06;
/***********************************************************************.
 *  aTp7() 
 ***********************************************************************/
void aTp7() {
  static double lpfAccelFpsOld = 0.0;
//  static double tp7OldSpeedError = 0.0;
  static double oldAccelFps = 0.0D;
 
  // Compute Center of Oscillation speed (cos)
  rotation3 = -gyroPitchDelta * (*currentValSet).t;  // 4.5
  cos3 = wFps + rotation3;
  // 0.92 .u value: 0.0 = no hf filtering, large values give slow response
  lpfCos3 = (lpfCos3Old * (*currentValSet).u) + (cos3 * (1.0D - (*currentValSet).u));
  lpfCos3Accel = lpfCos3 - lpfCos3Old;
  lpfCos3Old = lpfCos3;

  // Compute the acceleration speed
  double rad = (gaPitch + ZERO_ANGLE) * DEG_TO_RAD;
  double yG = ((double) lsm6.a.y) / ONE_G_Y;
  double zG = ((double) lsm6.a.z) / ONE_G_Z;
  double yAccel = -(((cos(rad) * yG) + (sin(rad) * zG)) * ACCEL_TO_FPS);
  accelFps += yAccel;

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
  coAccelFps = (coAccelFps * (*currentValSet).w) + (lpfCos3 * (1.0 - (*currentValSet).w));
//  lpfcoAccelFps -= lpfAccelFpsDelta;
//  lpfcoAccelFps = (lpfcoAccelFps * (*currentValSet).w) + (lpfCos3 * (1.0 - (*currentValSet).w));

  // Choose which of the computations to use.  Uncomment just one.
  tpFps = lpfCos3;
//  float tpFps = coAccelFps;
//  float tpFps = lpfcoAccelFps;

  if (isRouteInProgress)  tp7ControllerSpeed = routeFps;
  else tp7ControllerSpeed = controllerY * SPEED_MULTIPLIER; 

  // Find the speed error.  Constrain rate of change.
  double tp7SpeedError = tp7ControllerSpeed - tpFps;
//  double speedDiff = tp7SpeedError - tp7OldSpeedError;
//  if (speedDiff > 0.0D) tp7OldSpeedError += FPS_ACCEL;
//  else tp7OldSpeedError -= FPS_ACCEL;
//  tp7SpeedError = tp7OldSpeedError;

  // compute a weighted angle to eventually correct the speed error
  targetAngle = -(tp7SpeedError * (*currentValSet).x); //** 4.0 ******** Speed error to angle *******************
  
//targetAngle = 0.0;  
//fps = 0.0;
  // Compute maximum angles for the current wheel speed and enforce limits.
  targetAngle = constrain(targetAngle, -20.0, 20.0);

  // Compute angle error and weight factor
  angleError = targetAngle - gaPitch;
  fpsCorrection = angleError * (*currentValSet).y; // 0.4 ******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target wheel speed.
  targetWFps = fpsCorrection + tpFps;
//  targetWFps = fpsCorrection;

  // These routines set the steering values.
  if (isRouteInProgress) steerRoute(targetWFps);
  else steer(targetWFps);

} // end aTp7() 



/***********************************************************************.
 *  sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;
  
  if (isDumpingData) {
//    if (!(logLoop % 2))  dumpData();
    dumpData();
  }
  if (isDumpingTicks) {
    if ((logLoop % 4) == 0)  dumpTicks();
  }
  
//  if (!(logLoop % 104)) log2PerSec();
//  if (!(logLoop % 21)) log10PerSec();
//  routeLog(); //  208/sec
//  if (!(logLoop % 10)) log20PerSec(); // 20/sec  
//  if (!(logLoop % 2)) log104PerSec(); // 104/sec  
//  if (isRouteInProgress  && isRunning)  log208PerSec();
//  log208PerSec();
}

void log2PerSec() {
  Serial.print(isRunning); Serial.print(isRouteInProgress); Serial.println(isRunReady);
//  sprintf(message, "gyroHeading %4.2f   aPitch: %4.2f   gaPitch: %4.2f", gyroHeading, aPitch, gaPitch);
//  sendBMsg(SEND_MESSAGE, message);
//  Serial.print(forceLeft);
//  Serial.print(tab);
//  Serial.print(forceRight);
//  Serial.println();
}

void log10PerSec() {
  sprintf(message, "gaPitch %4.2f   gHeading: %4.2f   gcHeading: %4.2f", gaPitch, gHeading, gcHeading);
  Serial.println(message);
  sendBMsg(SEND_MESSAGE, message);
}

void log20PerSec() {
//  sprintf(message,  "%5.2f\t%5.2f\t%5.2f\t", sonarLeft, sonarFront, sonarRight);
//  sendBMsg(SEND_MESSAGE, message); 

  if (!isRunning) return;
  addLog(
       (long) (tickPosition),
        (short) (routeFps * 100.0),
        (short) (targetWFps * 100.0),
        (short) (tpFps * 100.0),
        (short) (wFps * 100.0),
        (short) (gaPitch * 100.0),
        (short) (currentLoc.y * 100.0)
    );
}

void log104PerSec() {
  addLog(
        (int) (tickPosition),
        (short) (lsm6.a.y),
        (short) (0 * 100.0),
        (short) (accelFps * 100.0),
        (short) (tpFps * 100.0),
        (short) (targetWFps * 100.0),
        (short) (0 * 100.0)
   );
}


void log208PerSec() {
  if (!isRouteInProgress) return;
  addLog(
        (long) (timeMilliseconds),
        (short) (gyroPitchRaw),
        (short) (gyroRollRaw),
        (short) (gyroYawRaw),
        (short) (gRoll * 100.0),
        (short) (aRoll * 100.0),
        (short) (0)
   );
}
        


/***********************************************************************.
 *  sendUpData() Send periodic loc, heading, & pitch to Up.
 ***********************************************************************/
void sendUpData() {
  static int sendCount = 0;
  if ((++sendCount % 4) == 0) {
    sendUMsg(TOUP_X, 2, currentLoc.x);
    sendUMsg(TOUP_Y, 2, currentLoc.y);
    sendUMsg(TOUP_HEADING, 2, gcHeading);
    sendUMsg(TOUP_PITCH, 0, gaPitch);
    sendUMsg(TOUP_FPS, 2, tpFps);
    sendUMsg(TOUP_DIST, 2, ((float) tickPosition) / TICKS_PER_FOOT);
  }
}


/***********************************************************************.
 *  steer() 
 ***********************************************************************/
void steer(float fp) {
  double speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX; 
  targetWFpsRight = fp - speedAdjustment;
  targetWFpsLeft = fp + speedAdjustment;
}


/***********************************************************************.
 *  steerRoute() X value from Up is the speed difference between the wheels.
 ***********************************************************************/
void steerRoute(float fp) {
  targetWFpsRight = fp - speedAdjustment;
  targetWFpsLeft = fp + speedAdjustment;
}

