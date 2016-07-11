/***********************************************************************.
 * ----- TP6 ------
 ***********************************************************************/
double tp6Fps = 0.0f;
double fpsCorrection = 0.0f;
//double tp6LoopSec = 0.0f;
double tp6LpfCosOld = 0.0;
//double targetHeading = 0.0;
double fpsLpfCorrectionOld = 0.0;
double fpsLpfCorrection = 0.0;
double tp6AngleError = 0.0;
double tp6TargetAngle = 0.0;
double tp6Cos = 0.0; 
double tp6Rotation = 0.0;
double rotationCorrection = 0.0;

/***********************************************************************.
 *  aTp6Run() 
 ***********************************************************************/
void aTp6Run() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  currentValSet = &tp6;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200);
//  readCompass();
  setHeading(0.0D);
  resetTicks();
  while(mode == MODE_TP6) { // main loop
    commonTasks();
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (isNewGyro()) {
      setGyroData();
      setNavigation();
      aTp6(); 
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
  tp6Rotation = (*currentValSet).t * (-gyroPitchDelta); // 3.6
  tp6Cos = wheelSpeedFps + tp6Rotation; // subtract rotation 
  tp6LpfCos = (tp6LpfCosOld * (1.0 - (*currentValSet).u))  + (tp6Cos  * (*currentValSet).u); // smooth it out a little (0.2)
//  tp6LpfCos = (tp6LpfCosOld * (1.0 - 0.05)) + (tp6Cos * 0.05); // smooth it out a little (0.2)
  tp6LpfCosAccel = tp6LpfCos - tp6LpfCosOld;
  tp6LpfCosOld = tp6LpfCos;

   // Do new calculation.  Used for including ticks in pitch estimation.
  rotation2 = -tgPitchDelta * 7.4;
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
    tp6TargetAngle = -(tp6SpeedError * (*currentValSet).v); //************ Speed error to angle *******************
  }
  
  // Compute maximum angles for the current wheel speed and enforce limits.
  float fwdA = wheelSpeedFps - 13.0;
  float bkwdA = wheelSpeedFps + 13.0;
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
//  tp6Fps = fpsLpfCorrection + tp6LpfCos;
  tp6Fps = fpsLpfCorrection + lpfCos2;

  // These routines set the steering values, amoung other things.
  if (isRouteInProgress) route();
  else if (isStand) standSteer();
  else tp6Steer(tp6Fps);
//  usec(
//        (long) timeMicroseconds,
//        (short) (gaPitch * 100.0),
//        (short) (tp6ControllerSpeed * 10.0),
//        (short) (lpfCos2 * 100.0),
//        (short) (tp6SpeedError * 100.0),
//        (short) (tp6Fps * 100.0),
//        (short) (currentMapLoc.y * 100.0)
//   );
//
  setTargetSpeedRight(tp6FpsRight);
  setTargetSpeedLeft(tp6FpsLeft);
} // end aTp6() 



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

  if ((logLoop % 42) == 5) { // 10/sec
  }
  if ((logLoop % 21) == 7) { // 20/sec
   routeLog();
  }    
}



void log2PerSec() {
//    sprintf(message, "gPitch %4.2f   aPitch: %4.2f   gaPitch: %4.2f", gPitch, aPitch, gaPitch);
//    sendBMsg(SEND_MESSAGE, message);
//  Serial.print(tp6FpsLeft);
//  Serial.print(tab);
//  Serial.print(tp6FpsRight);
//  Serial.println();
}



void log20PerSec() {
//  snprintf(pBuf, sizeof(pBuf), "%5d", (int) gyroCumHeading);
//  sendBMsg(SEND_MESSAGE, pBuf); 

  if (!isRouteInProgress) return;
  addLog(
        (long) coTickPosition,
        (short) (currentMapCumHeading * 10.0),
        (short) (turnTargetCumHeading * 10.0),
        (short) (wheelSpeedFps * 100.0),
        (short) (routeStepPtr),
        (short) (currentMapLoc.x * 100.0),
        (short) (currentMapLoc.y * 100.0)
   );
}

void log400PerSec() {
//  if (!isRouteInProgress) return;
  addLog(
        (long) (0),
        (short) (currentMapLoc.y * 100.0),
        (short) (tp6LpfCos * 100.0),
        (short) (gaPitch * 100.0),
        (short) (wheelSpeedFps * 100.0),
        (short) (stopDist * 100.0),
        (short) (routeStepPtr)
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
  tp6FpsLeft = tp6Fps + speedAdjustment;
  tp6FpsRight = tp6Fps - speedAdjustment;
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

  tp6FpsRight = tp6Fps - headingSpeedAdjustment;
  tp6FpsLeft = tp6Fps + headingSpeedAdjustment;
}


