/***********************************************************************.
 * ----- TP6 ------
 ***********************************************************************/
float fps = 0.0;
float accelFps = 0.0;
//float targetFps = 0.0f;
float fpsCorrection = 0.0f;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float angleError = 0.0;
float targetAngle = 0.0;


/***********************************************************************.
 *  aTp7Run() 
 ***********************************************************************/
void aTp7Run() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  currentValSet = &tp7;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200); // For switches?
  setHeading(0.0D);
  resetTicks();
  beep(BEEP_UP);
  while(mode == MODE_2P) { // main loop
    if (isNewCheck) {
      isNewCheck = false;
      Serial.println(debugFloat1);
    }
    commonTasks();
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (isNewGyro()) {
      setGyroData();
      setNavigation();
      aTp7(); 
      sendLog();
      safeAngle();
    } else if (isNewAccel()) {
      setAccelData();
    }
  }
}



const int Y_BIAS = 520;
const float Y_ONE_G = -16500.0;
const float ACCEL_TO_SPEED = 0.000008;
const float COMP_TC = 0.98;
const float ACCEL_TC = 0.90;
//const float COMP_TC = 0.0; // Cos only
const double FPS_ACCEL = 0.05;
/***********************************************************************.
 *  aTp7() 
 ***********************************************************************/
void aTp7() {
  static float lpfAccelFpsOld = 0.0;
  static double tp7OldSpeedError = 0.0;

  // Compute Center of Oscillation speed (cos)
  rotation3 = -gyroPitchDelta * (*currentValSet).t;  // 4.5
  cos3 = wFps + rotation3;
  // 0.95 .u value: 0.0 = no hf filtering, large values give slow response
  lpfCos3 = (lpfCosOld3 * (*currentValSet).u) + (cos3 * (1.0D - (*currentValSet).u));
  lpfCosOld3 = lpfCos3;

  // Compute the acceleration speed
  float yRawAccel = (float) (lsm6.a.y - Y_BIAS); // Subtract out the constant error.
  float yAccel = yRawAccel - (sin(gaPitch * DEG_TO_RAD) * Y_ONE_G); // Subtract out gravity part
  float accelDelta = yAccel * ACCEL_TO_SPEED;  // keep for debugging
  accelFps -= accelDelta;   // get the speed from the accelerometer
  if (!isRunning) accelFps = lpfAccelFpsOld = 0.0;
  float lpfAccelFps =(lpfAccelFpsOld * (*currentValSet).v) + (accelFps * (1.0 - (*currentValSet).v)); // 0.9
  fps += lpfAccelFps - lpfAccelFpsOld;
  lpfAccelFpsOld = lpfAccelFps;
//  fps -= accelDelta;

  // Complementary filter with COS. Try to minimize loss of traction effects.
  // 0.98, High value places more emphasis on accel.
  fps = (fps * (*currentValSet).w) + (lpfCos3 * (1.0 - (*currentValSet).w));

  if (isRouteInProgress) tp7ControllerSpeed = routeFps;
  else tp7ControllerSpeed = controllerY * SPEED_MULTIPLIER; 

  // Find the speed error.  Constrain rate of change.
  double tp7SpeedError = tp7ControllerSpeed - fps;
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
//  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - ((*currentValSet).x)))  + (fpsCorrection * ((*currentValSet).x));
//  fpsLpfCorrectionOld = fpsLpfCorrection;

  // Add the angle error to the base speed to get the target wheel speed.
  targetWFps = fpsCorrection + fps;
//  targetWFps = fpsCorrection;

  // These routines set the steering values, among other things.
  if (isRouteInProgress) steerRoute();
  else steer(targetWFps);
  if (isRunning) {
    addLog(
        (long) timeMilliseconds,
        (short) (wFps * 100.0),
        (short) (rotation3 * 100.0),
        (short) (lpfCos3 * 100.0),
        (short) (accelFps * 100.0),
        (short) (0.0 * 100.0),
        (short) (targetWFps * 100.0)
    );
  }
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
  
//  if ((logLoop % 104) == 5) log2PerSec();
//  if ((logLoop % 21) == 5) ;  // 10/sec
//  if ((logLoop % 10) == 0) routeLog(); //  
//  if ((logLoop % 10) == 7) log20PerSec(); // 20/sec  
//  if (!(logLoop % 2)) log104PerSec(); // 104/sec  
//  if (isRouteInProgress  && isRunning)  log208PerSec();
}

void log2PerSec() {
  sprintf(message, "gPitch %4.2f   aPitch: %4.2f   gaPitch: %4.2f", gPitch, aPitch, gaPitch);
  sendBMsg(SEND_MESSAGE, message);
  Serial.print(forceLeft);
  Serial.print(tab);
  Serial.print(forceRight);
  Serial.println();
}

void log20PerSec() {
  sprintf(message,  "%5.2f\t%5.2f\t%5.2f\t", sonarLeft, sonarFront, sonarRight);
  sendBMsg(SEND_MESSAGE, message); 

  if (!isRunning) return;
  addLog(
        (long) coTickPosition,
        (short) (gyroCumHeading * 10.0),
        (short) (wMFpsLeft),
        (short) (wMFpsRight),
        (short) (wFps * 100.0),
        (short) (currentLoc.x * 100.0),
        (short) (currentLoc.y * 100.0)
   );
}

void log104PerSec() {
  addLog(
        (int) (tickPosition),
        (short) (lsm6.a.y),
        (short) (0 * 100.0),
        (short) (accelFps * 100.0),
        (short) (fps * 100.0),
        (short) (targetWFps * 100.0),
        (short) (0 * 100.0)
   );
}


void log416PerSec() {
  addLog(
        (long) (tickPosition),
        (short) (routeFps * 100.0),
        (short) (targetWFps * 100.0),
        (short) (gaPitch * 100.0),
        (short) (gPitch * 100.0),
        (short) (tPitch * 100.0),
        (short) (tgPitch * 100.0)
   );
}
        


/***********************************************************************.
 *  steer() 
 ***********************************************************************/
void steer(float fp) {
  double speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX; 
  targetWFpsRight = fp - speedAdjustment;
  targetWFpsLeft = fp + speedAdjustment;
}



/************************************************************************
 *  spin() Just keep the motors running correctly while doing a spin.
 ************************************************************************/
double spin() {
//  if (isSpin) {
//    double spinRate = controllerX * 10.0D;
//    setTargetSpeedRight(-spinRate);
//    setTargetSpeedLeft(spinRate);
//  }
//  else {
//    double normalRate = (((1.0 - abs(0.0)) * 1.5) + 0.5) * controllerX; 
//    setTargetSpeedRight(-normalRate);
//    setTargetSpeedLeft(normalRate);
//  }
}



