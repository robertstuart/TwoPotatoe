/***********************************************************************.
 * ----- TP6 ------
 ***********************************************************************/
float tp7Fps = 0.0;
float accelFps = 0.0;
double targetFps = 0.0f;
float tp7TargetFps = 0.0f;
double fpsCorrection = 0.0f;
float tp7FpsCorrection = 0.0f;
//double tp6LoopSec = 0.0f;
double tp6LpfCosOld = 0.0;
//double targetHeading = 0.0;
double fpsLpfCorrectionOld = 0.0;
double fpsLpfCorrection = 0.0;
double tp6AngleError = 0.0;
float tp7AngleError = 0.0;
double tp6TargetAngle = 0.0;
float tp7TargetAngle = 0.0;
double tp6Cos = 0.0; 
double tp6Rotation = 0.0;
double rotationCorrection = 0.0;

/***********************************************************************.
 *  aTp6Run() 
 ***********************************************************************/
unsigned int t0, t1, t2, t3;
void aTp6Run() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  currentValSet = &tp6;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200);   // For switches?
//  readCompass();
  setHeading(0.0D);
  resetTicks();
  while(mode == MODE_2P) { // main loop
    commonTasks();
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (isNewGyro()) {
      setGyroData();
      setNavigation();
      aTp6(); 
//      aTp7(); 
      sendLog();
      safeAngle();
      checkMotorRight();
      checkMotorLeft();
    }
    if (isNewAccel()) {
      setAccelData();
//      aTp7(); 
    }
  }
}
/***********************************************************************.
 *  aTp7Run() 
 ***********************************************************************/
void aTp7Run() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200); // For switches?
  setHeading(0.0D);
  resetTicks();
  while(mode == MODE_2P) { // main loop
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
      checkMotorRight();
      checkMotorLeft();
    }
    if (isNewAccel()) {
      setAccelData();
    }
  }
}



/***********************************************************************.
 *  aTp6() 
 ***********************************************************************/
void aTp6() {
  readSpeed();
  // compute the Center of Oscillation Speed (COS)
  tp6Rotation = 6.4 * (-gyroPitchDelta); // 6.4
//  tp6Rotation = 6.4 * (-tgPitchDelta); // 6.4
  tp6Cos = wheelSpeedFps + tp6Rotation; // subtract rotation 
  tp6LpfCos = (tp6LpfCosOld * (1.0 - (*currentValSet).u))  + (tp6Cos  * (*currentValSet).u); // smooth it out a little (0.2)
//  tp6LpfCos = (tp6LpfCosOld * (1.0 - 0.05)) + (tp6Cos * 0.05); // smooth it out a little (0.2)
  tp6LpfCosAccel = tp6LpfCos - tp6LpfCosOld;
  tp6LpfCosOld = tp6LpfCos;

   // Do new calculation.  Used for including ticks in pitch estimation.
  rotation2 = -tgPitchDelta * (*currentValSet).t;  // 4.5
  cos2 = wheelSpeedFps + rotation2;
  lpfCos2 = (lpfCosOld2 * .9) + (cos2 * (1.0D - .9));
  lpfCosOld2 = lpfCos2;

  if (isRouteInProgress) {
    tp6ControllerSpeed = routeFps;
  }
  else if (isStand) {
    tp6ControllerSpeed = standFps();
  }
  else {
    tp6ControllerSpeed = controllerY * SPEED_MULTIPLIER; 
  }

  // find the speed error
//  double tp6SpeedError = tp6ControllerSpeed - tp6LpfCos;
  double tp6SpeedError = tp6ControllerSpeed - lpfCos2;

  // compute a weighted angle to eventually correct the speed error
  if (!isAngleControl) { // TargetAngle set by route() routines?
    tp6TargetAngle = -(tp6SpeedError * (*currentValSet).v); //** 4.0 ******** Speed error to angle *******************
  }
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  float fwdA = wheelSpeedFps - 20.0;
  float bkwdA = wheelSpeedFps + 20.0;
  if (tp6TargetAngle < fwdA)  tp6TargetAngle = fwdA;
  if (tp6TargetAngle > bkwdA) tp6TargetAngle = bkwdA;
//  double tp6TargetAngle = tp6SpeedError * 2.0; //********** Speed error to angle *******
  
  // Compute angle error and weight factor
//  tp6AngleError = tp6TargetAngle - gaPitch + rotationCorrection;  //** 2
  tp6AngleError = tp6TargetAngle - gaPitch;  //** 2
  fpsCorrection = tp6AngleError * (*currentValSet).w; //******************* Angle error to speed *******************
//  speedCorrection = tp6AngleError * 0.18; //******************* Angle error to speed *******************
//  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - 0.1))  + (speedCorrection * 0.1);
  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - ((*currentValSet).x)))  + (fpsCorrection * ((*currentValSet).x));
  fpsLpfCorrectionOld = fpsLpfCorrection;

  // Add the angle error to the base speed to get the target speed.
//  targetFps = fpsLpfCorrection + tp6LpfCos;
  targetFps = fpsLpfCorrection + lpfCos2;

  // These routines set the steering values, amoung other things.
  if (isRouteInProgress) route();
  else if (isStand) standSteer();
  else tp6Steer(targetFps);
  setTargetSpeedRight(targetFpsRight);
  setTargetSpeedLeft(targetFpsLeft);
} // end aTp6() 


const int Y_BIAS = 520;
const float Y_ONE_G = -16500.0;
const float ACCEL_TO_SPEED = 0.00001;
const float ACCEL_TC = 0.90;
const double FPS_ACCEL = 0.001;
/***********************************************************************.
 *  aTp7() 
 ***********************************************************************/
void aTp7() {
  static double tp7OldSpeedError = 0.0D;
  // Do the speed calculations.
  readSpeed();

  // Compute Center of Oscillation speed (cos)
  rotation3 = -gyroPitchDelta * 4.5;  // 4.5
  cos3 = wheelSpeedFps + rotation3;
  lpfCos3 = (lpfCosOld3 * .9) + (cos3 * (1.0D - .9));
  lpfCosOld3 = lpfCos3;

  // Compute the acceleration speed
  float yRawAccel = (float) (lsm6.a.y - Y_BIAS); // Subtract out the constant error.
  float yAccel = yRawAccel - (sin(gaPitch * DEG_TO_RAD) * Y_ONE_G); // Subtract out gravity part
  accelFps -= yAccel * ACCEL_TO_SPEED * 0.25; // Keep track of drift for debugging.
  tp7Fps -= yAccel * ACCEL_TO_SPEED * 0.25;   // divide by 4 for 416/sec
  if (!isRunning) accelFps = tp7Fps = 0.0;

  // Complementary filter with COS. Try to minimize loss of traction effects.
//  if ((tp7Fps - cos3) > 1.0) lpfCos3 = tp7Fps - 1.0;
//  if ((tp7Fps - cos3) < -1.0) lpfCos3 = tp7Fps + 1.0;
  tp7Fps = (tp7Fps * ACCEL_TC) + (lpfCos3 * (1.0 - ACCEL_TC));

  if (isRouteInProgress) tp7ControllerSpeed = routeFps;
  else tp7ControllerSpeed = controllerY * SPEED_MULTIPLIER; 

  // find the speed error
  double tp7SpeedError = tp7ControllerSpeed - tp7Fps;
  double speedDiff = tp7SpeedError - tp7OldSpeedError;
  if (speedDiff > 0.0D) tp7OldSpeedError += FPS_ACCEL;
  else tp7OldSpeedError -= FPS_ACCEL;
  tp7SpeedError = tp7OldSpeedError;
  

  // compute a weighted angle to eventually correct the speed error
  tp7TargetAngle = -(tp7SpeedError * (*currentValSet).v); //** 4.0 ******** Speed error to angle *******************
  
  // Compute maximum angles for the current wheel speed and enforce limits.

  // Compute angle error and weight factor
  tp7AngleError = tp7TargetAngle - gaPitch;
  tp7FpsCorrection = tp7AngleError * (*currentValSet).w; //******************* Angle error to speed *******************
//  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - ((*currentValSet).x)))  + (fpsCorrection * ((*currentValSet).x));
//  fpsLpfCorrectionOld = fpsLpfCorrection;

  // Add the angle error to the base speed to get the target speed.
  tp7TargetFps = tp7FpsCorrection + tp7Fps;
//  targetFps = tp7TargetFps = tp7FpsCorrection + tp7Fps;
// old  targetFps = fpsLpfCorrection + lpfCos2;

  // These routines set the steering values, amoung other things.
//  if (isRouteInProgress) route();
//  else if (isStand) standSteer();
//  else tp6Steer(targetFps);
//  setTargetSpeedRight(targetFpsRight);
//  setTargetSpeedLeft(targetFpsLeft);
} // end aTp7() 



/***********************************************************************.
 *  sendLog() Called 416 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;
  
  if (isDumpingData) {
    if ((logLoop % 4) == 0)  dumpData();
  }
  if (isDumpingTicks) {
    if ((logLoop % 4) == 0)  dumpTicks();
  }
  
  if ((logLoop % 208) == 5) log2PerSec();
//  if ((logLoop % 42) == 5) ;  // 10/sec
//  if ((logLoop % 21) == 0) routeLog(); //  
//  if ((logLoop % 21) == 7) log20PerSec(); // 20/sec  
  if (!(logLoop % 4)) log104PerSec(); // 100/sec  
//  if (isRouteInProgress  && isRunning)  log416PerSec();
}

void log2PerSec() {
//  sprintf(message, "gPitch %4.2f   aPitch: %4.2f   gaPitch: %4.2f", gPitch, aPitch, gaPitch);
//  sendBMsg(SEND_MESSAGE, message);
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
        (short) (mFpsLeft),
        (short) (mFpsRight),
        (short) (wheelSpeedFps * 100.0),
        (short) (currentLoc.x * 100.0),
        (short) (currentLoc.y * 100.0)
   );
}

void log104PerSec() {
  addLog(
        (int) (tickPosition),
        (short) (lsm6.a.y),
        (short) (tp6Cos * 100.0),
        (short) (accelFps * 100.0),
        (short) (tp7Fps * 100.0),
        (short) (targetFps * 100.0),
        (short) (tp7TargetFps * 100.0)
   );
}


void log416PerSec() {
  addLog(
        (long) (tickPosition),
        (short) (routeFps * 100.0),
        (short) (targetFps * 100.0),
        (short) (gaPitch * 100.0),
        (short) (gPitch * 100.0),
        (short) (tPitch * 100.0),
        (short) (tgPitch * 100.0)
   );
}
        


/***********************************************************************.
 *  tp6Steer() 
 ***********************************************************************/
void tp6Steer(double fps) {
  double speedAdjustment;
  if (isSpin) {
    speedAdjustment = controllerX * 10.0D;
  }
  else {
    speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX; 
  }
  targetFpsLeft = targetFps + speedAdjustment;
  targetFpsRight = targetFps - speedAdjustment;
}



/************************************************************************
 *  spin() Just keep the motors running correctly while doing a spin.
 ************************************************************************/
double spin() {
  if (isSpin) {
    double spinRate = controllerX * 10.0D;
    setTargetSpeedRight(-spinRate);
    setTargetSpeedLeft(spinRate);
  }
  else {
    double normalRate = (((1.0 - abs(0.0)) * 1.5) + 0.5) * controllerX; 
    setTargetSpeedRight(-normalRate);
    setTargetSpeedLeft(normalRate);
  }
}



/************************************************************************
 *  stand() Keep position from base tickPosition
 ************************************************************************/
double standFps() {
  routeFps = 0.0;
  
  if (abs(controllerY) > 0.05) {
    standTPRight = tickPositionRight;
    standTPLeft = tickPositionLeft;
    return(controllerY * 1.0);
  } 
  else {
    int targetPos = standTPRight + standTPLeft;
    int currentPos = tickPositionRight + tickPositionLeft;
    return((float) ((targetPos - currentPos)) * 0.0005);
  }
}

void standSteer() {
  float headingSpeedAdjustment = 0.0;
   
  if (abs(controllerX) > 0.05) {
    headingSpeedAdjustment = controllerX * 0.3;
    standTPRight = tickPositionRight;
    standTPLeft = tickPositionLeft;
  }
  else {
    int targetTD = standTPRight - standTPLeft;
    int currentTD = tickPositionRight - tickPositionLeft;
    headingSpeedAdjustment = ((float) (currentTD - targetTD)) * 0.01;
  }

  targetFpsRight = targetFps - headingSpeedAdjustment;
  targetFpsLeft = targetFps + headingSpeedAdjustment;
}


