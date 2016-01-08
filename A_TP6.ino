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
  unsigned int subCycle = 0;
  
  timeMicroseconds = gyroTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  currentValSet = &tp6;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200);
  readCompass();
  setHeading(0.0D);
  resetTicks();
  while(mode == MODE_TP6) { // main loop
    commonTasks();
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (timeMicroseconds > gyroTrigger) {
      gyroTrigger +=  2400; // ~400/sec
      subCycle++;
      setControllerXY();
      readGyro();
      if ((subCycle % 16) == 3) readCompass();   // 25/sec
      setNavigation();
      if ((subCycle % 4)  == 1) readAccel();  // 100/sec
      aTp6(); 
      sendTp6Status();
      safeAngle();
      switches();
      checkMotorRight();
      checkMotorLeft();
    } // end timed 400/sec loop
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

  
//  if (isJump) {
//    setTargetSpeedRight(tp6FpsRightJump);
//    setTargetSpeedLeft(tp6FpsLeftJump);
//  }
//  else {
    setTargetSpeedRight(tp6FpsRight);
    setTargetSpeedLeft(tp6FpsLeft);
//  }
} // end aTp6() 



/***********************************************************************.
 *  sendTp6Status() Called 400 time/sec.
 ***********************************************************************/
void sendTp6Status() {
  static unsigned int loopc = 0;
  static unsigned int loopd = 0;
  static float marker = 1.1;
  loopc = ++loopc % 40; // 10/sec loop.
  loopd = ++loopd % 400; // 1/sec loop.
  if (isDumpingData) {
    if ((loopc % 4) == 0)  dumpData();
  }
  if (isDumpingTicks) {
    if ((loopc % 4) == 0)  dumpTicks();
  }
  if ((loopc == 0)  || (loopc == 20))  { // Status 20/sec
    sendStatusXBeeHc(); 
    sendStatusBluePc();
    isNewMessage = false;
  }
  else if ((loopc == 10) || (loopc == 30)) { // Logging & debugging 20/sec
//    log20PerSec();
//  sprintf(message,"%7.3f %7.3f %7.3f %7.3f %7d %7.3f", 
//          xVec, yVec, zVec, tmCumHeading, mY, magHeading);
//   sendBMsg(SEND_MESSAGE, message);
    if (isRouteInProgress) routeLog();
//    routeLog();
  }    
//  if (loopd == 0) log1PerSec();
//  log400PerSec();
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
        
void log1PerSec() {
  static unsigned long t;
  unsigned long t2 = millis();
  int tickDiff = tickPositionLeft - tickPositionRight;
  double tickA = ((double) tickDiff) / TICKS_PER_DEGREE_YAW;
  sprintf(message, "tickDiff:%5d     tickAngle:%6.2f", tickDiff , tickA);
  sendBMsg(SEND_MESSAGE, message); 
}

/***********************************************************************.
 *  tp6Steer() 
 ***********************************************************************/
void tp6Steer(double fps) {
  double speedAdjustment;
  controllerX = getControllerX();
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
    double spinRate = getControllerX() * 10.0D;
    setTargetSpeedRight(-spinRate);
    setTargetSpeedLeft(spinRate);
  }
  else {
//    double normalRate = getControllerX() * 2.0D;
    double normalRate = (((1.0 - abs(0.0)) * 1.5) + 0.5) * getControllerX(); 
    setTargetSpeedRight(-normalRate);
    setTargetSpeedLeft(normalRate);
  }
}



/************************************************************************
 *  stand() Keep position from base tickPosition
 ************************************************************************/
double standFps() {
  float joyY = (abs(pcY) > abs(hcY)) ? pcY : hcY;
  routeFps = 0.0;
  
  if (abs(joyY) > 0.05) {
    standTPRight = tickPositionRight;
    standTPLeft = tickPositionLeft;
    return(joyY * 1.0);
  } 
  else {
    int targetPos = standTPRight + standTPLeft;
    int currentPos = tickPositionRight + tickPositionLeft;
    return((float) ((targetPos - currentPos)) * 0.0005);
  }
}

void standSteer() {
  float headingSpeedAdjustment = 0.0;
  float joyX = (abs(pcX) > abs(hcX)) ? pcX : hcX;
  
  if (abs(joyX) > 0.05) {
    headingSpeedAdjustment = joyX * 0.3;
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


