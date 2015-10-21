/***********************************************************************.
 * ----- TP6 ------
 ***********************************************************************/
double tp6Fps = 0.0f;
double fpsCorrection = 0.0f;
double tp6ControllerSpeed = 0;
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
//  angleInit6();  // causes freeze?
//  motorInitTp6();
  motorInitTp();
  currentValSet = &tp6;
  setBlink(RED_LED_PIN, BLINK_SB);
  delay(200);
  readCompass();
  resetNavigation('M', 0.0);
  while(mode == MODE_TP6) { // main loop
    commonTasks();
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (timeMicroseconds > gyroTrigger) {
      gyroTrigger +=  2400; // 400/sec
      subCycle++;
      readGyro();
      if ((subCycle % 4)  == 1) readAccel();  // 100/sec
      if ((subCycle % 16) == 3) readCompass();   // 25/sec
      setNavigation();
      setControllerXY();
      aTp6(); 
      sendTp6Status();
      safeAngle();
      switches();
      checkMotorRight();
      checkMotorLeft();
//      log6();
    } // end timed loop
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

  // compute the Center of Oscillation Tick Position
  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

  // Do new calculation.  Used for 
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
  
  // Correct for angle error during rotation on vertical axis.  Do not know the cause of this.
  // Is it the actual angle or is it an error from the acceleromenter?
  // Seems to work correctly if it is lp filtered (at accel rate?).
  // Should this be done in Angle6?
  double rc = (fpsRight - fpsLeft) * 1.9;
  rotationCorrection = ((rc - rotationCorrection) * .005) + rotationCorrection;
  
  // Compute angle error and weight factor
  tp6AngleError = tp6TargetAngle - gaPitch + rotationCorrection;  //** 2
//  tp6AngleError = tp6TargetAngle - gaPitch;  //** 2
  fpsCorrection = tp6AngleError * (*currentValSet).w; //******************* Angle error to speed *******************
//  speedCorrection = tp6AngleError * 0.18; //******************* Angle error to speed *******************
//  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - 0.1))  + (speedCorrection * 0.1);
  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - ((*currentValSet).x)))  + (fpsCorrection * ((*currentValSet).x));
  fpsLpfCorrectionOld = fpsLpfCorrection;

  // Add the angle error to the base speed to get the target speed.
//  tp6Fps = fpsLpfCorrection + tp6LpfCos;
  tp6Fps = fpsLpfCorrection + lpfCos2;
  
  if (isRouteInProgress) route();
  else if (isStand) standSteer();
  else tp6Steer(tp6Fps);
  
  if (isJump) {
    setTargetSpeedRight(tp6FpsRightJump);
    setTargetSpeedLeft(tp6FpsLeftJump);
  }
  else {
    setTargetSpeedRight(tp6FpsRight);
    setTargetSpeedLeft(tp6FpsLeft);
  }
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
  if ((loopc == 0)  || (loopc == 20))  { // Status 20/sec
    sendStatusXBeeHc(); 
    sendStatusBluePc();
    isNewMessage = false;
  }
  else if ((loopc == 10) || (loopc == 30)) { // debugging print statements 20/sec
    if (isRouteInProgress) {
//      routeLog();
    }
  }
    
  if (loopd == 0) {
//    Serial.print(currentMapHeading); Serial.print("\t"); Serial.println(magHeading);
//    Serial.print(forceRight); Serial.print("\t"); Serial.println(forceLeft);
//Serial.print(currentMapHeading);Serial.print("\t");Serial.print(magHeading);Serial.print("\t");Serial.println(gyroHeading);
  }
  
    // Every loop (400/sec)
    addLog(
          (long) coTickPosition,
          (short) (wheelSpeedFps * 100.0),
          (short) (gaPitch * 100.0),
          (short) (tp6FpsRightJump * 100.0),
          (short) (isOnGround),
          (short) (isJump),
          (short) (tp6LpfCos * 100.0)
     );
}


///***********************************************************************.
// *  log6() - Put data in circular log buffer.
// ***********************************************************************/
//void log6() {
//  noInterrupts();
//  if (motorStateRight) {
//    timeMicroseconds = micros();
//    unsigned int t = timeMicroseconds - startTimeRight;
//    if (signPwRight > 0) pwPlusSumRight += t;
//    else pwMinusSumRight += t;
//    startTimeRight = timeMicroseconds;
//  }
//  unsigned int pP = pwPlusSumRight;
//  unsigned int pM = pwMinusSumRight;
//  int p = pP - pM;
//  pwPlusSumRight = 0;
//  pwMinusSumRight = 0;
//  interrupts();
//  
////  addLog(
////          (long) (gaPitch * 100.0),
////          (short) pP,
////          (short) pM,
////          (short) (wheelSpeedFps * 100.0),
////          (short) (tp6LpfCos * 100.0),
////          (short) (tp6TargetAngle * 100.0),
////          (short) (fpsLpfCorrection * 100.0));
//}
//
//

/***********************************************************************.
 *  tp6Steer() 
 ***********************************************************************/
void tp6Steer(double fps) {
  double speedAdjustment;
  controllerX = getControllerX();
  if (!isGyroSteer) {
      speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX;
    }
    else {
      targetGHeading += 0.1 * controllerX;
      double aDiff =  targetGHeading - gyroCumHeading;
      speedAdjustment = aDiff * (S_LIM / A_LIM);
      speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
    }
    tp6FpsLeft = tp6Fps + speedAdjustment;
    tp6FpsRight = tp6Fps - speedAdjustment;
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


