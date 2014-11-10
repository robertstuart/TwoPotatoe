/************************************************************************
 * ----- TP5 ------
 ************************************************************************/
float tp5RawIError = 0.0f; 
float tp5ErrorSpeedFps = 0.0f;
float tp5ISpeed = 0.0f;
float tp5Fps = 0.0f;
float tp5CoSpeed = 0.0f;
float tp5OldCospeed = 0.0f; 
float tp5AngleErrorW = 0.0f;
boolean tp5IsHome = false;
//long tp5HomeDistance = 0L;
float tp5HomeSpeed = 0.0f;
long tp5AccumDistanceError = 0l;
float tp5ControllerSpeed = 0;
float tp5LoopSec = 0.0f;
unsigned int tp5LoopCounter = 0;
float tp5LpfCos = 0.0;
float tp5LpfCosOld = 0.0;
float oldGaPitchAngle = 0.0;
long routeResetTime = 0L;
boolean clockwise = true;
float targetHeading = 0.0;
long targetTickDistance = 0L;
float turnSpeedAdjustment = 0.0;
float tp5LpfAngleErrorWOld = 0.0;
float tp5LpfAngleErrorW = 0.0;
float tp5AngleError = 0.0;
float dampValue = 0.0;
float zeroAcc = 0.0;

float tp5LpfCosLeft = 0.0f;
float tp5LpfCosLeftOld = 0.0f;


/************************************************************************
 *  aTp5Run() 
 ************************************************************************/
void aTp5Run() {
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  tp5RawIError = 0.0f;
  motorInitTp();
//  angleInitTp7();
  while(mode == MODE_TP5) { // main loop
    commonTasks();
//    route();

    // Do the timed loop
    if (readImu()) {
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      oldTimeTrigger = timeMicroseconds;
      tp5LoopSec = ((float) actualLoopTime)/1000000.0; 
      aTp5(); 
      sendTp5Status();
      magTickCorrection();
      safeAngle();
      gravity();
      runSwitch();
      checkMotorRight();
      checkMotorLeft();
//    checkDrift();
    } // end timed loop
  }
}



/************************************************************************
 *  aTp5() 
 ************************************************************************/
void aTp5() {
  readSpeed();
  timeMicroseconds = micros(); // So algorithm will have latest time
  getTp5Angle();
//  float tp5AngleDelta = gaPitchAngle - oldGaPitchAngle; //** 2
//  oldGaPitchAngle = gaPitchAngle; //** 2
  
  float tp5AngleDelta = gyroPitchAngleDelta; //** 2

  // compute the Center of Oscillation Speed (COS)
  float tp5Cos = wheelSpeedFps + ((*currentValSet).v * tp5AngleDelta); // subtract out rotation **************
  tp5LpfCos = tp5LpfCosOld + ((tp5Cos - tp5LpfCosOld) * (*currentValSet).w); // smooth it out a little (0.2)
  tp5LpfCosAccel = tp5LpfCos - tp5LpfCosOld;
  tp5LpfCosOld = tp5LpfCos;

//runLog((long) (tp5LpfCosAccel * 1000.0), (long) aPitch , (long) (gaPitchAngle * 1000.0), (long) 0); // for calibration
runLog((long) (wheelSpeedFps * 1000.0), (long) (tp5Fps * 1000.0), (long) (gaPitchAngle * 1000.0), 0L);
//Serial.print(sState); Serial.print(cState); Serial.println(tState);
//Serial.print(abs(gaPitchAngle)); Serial.print("\t"); Serial.println((abs(gaRollAngle) < 35.0));
Serial.print(aPitch); Serial.print("\t"); Serial.println(aPitchRoll);
//  // newCos
//  int newCos = mWheelSpeedFps + (gyroPitchRaw / 10); // subtract out rotation **************
//  newLpfCos = newLpfCosOld + (((newCos - newLpfCosOld) * 20) / 100); // smooth it out a little
//  newLpfCosOld = newLpfCos;
//
//  // newCos left
//  int newCosLeft = mAverageFpsLeft + (gyroPitchRaw / 10); // subtract out rotation **************
//  newLpfCosLeft = newLpfCosLeftOld + (((newCosLeft - newLpfCosLeftOld) * 20) / 100); // smooth it out a little
//  newAccelLeft = newLpfCosLeft - newLpfCosLeftOld;
//  newLpfCosLeftOld = newLpfCosLeft;
//
//  // compute the Center of Oscillation Speed (COS) for just the left side
//  float tp5CosLeft = fpsLeft + ((*currentValSet).v * tp5AngleDelta); // subtract out rotation **************
//  tp5LpfCosLeft = tp5LpfCosLeftOld + ((tp5CosLeft - tp5LpfCosLeftOld) * (*currentValSet).w); // smooth it out a little
//  accelLeft = tp5LpfCosLeft - tp5LpfCosLeftOld;
//  tp5LpfCosLeftOld = tp5LpfCosLeft;
//  
  

  tp5ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp5SpeedError = tp5ControllerSpeed - tp5LpfCos;
//  float tp5SpeedError = tp5ControllerSpeed - (((float) newCos) / 1000.0);

  // compute a weighted angle to eventually correct the speed error
  float tp5TargetAngle = tp5SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  tp5AngleError = gaPitchAngle - tp5TargetAngle;  //** 2

  // Original value for y: 0.09
  tp5AngleErrorW = tp5AngleError * (*currentValSet).y; //******************* Angle error to speed *******************
  tp5LpfAngleErrorW = tp5LpfAngleErrorWOld + ((tp5AngleErrorW - tp5LpfAngleErrorWOld) * 0.1);
  tp5LpfAngleErrorWOld = tp5LpfAngleErrorW;

  // Add the angle error to the base speed to get the target speed.
  tp5Fps = tp5LpfAngleErrorW + tp5LpfCos;
//  if (damp()) {
//    tp5Fps = tp5LpfCos;
//  }

  tp5Steer(tp5Fps);
  setTargetSpeedRight(tp5FpsRight);  // Need to set direction.  Does not set speed!
  setTargetSpeedLeft(tp5FpsLeft);  // Need to set direction.  Does not set speed!
  
//  // Set the values for the interrupt routines
//  targetTDR = tickDistanceRight + (15.0 * tp5AngleError);  // Target beyond current tickDistance.
//  targetTDL = tickDistanceLeft + (15.0 * tp5AngleError);  // Target beyond current tickDistance.
//  fpsRightLong = (long) (tp5FpsRight * 100.0);
//  fpsLeftLong = (long) (tp5FpsLeft * 100.0);
//  loopTickDistanceR = tickDistanceRight;
//  loopTickDistanceL = tickDistanceLeft;
//  tp5LoopTimeR = tickTimeRight;
//  tp5LoopTimeL = tickTimeLeft;
  
} // end aTp5() 



/************************************************************************
 *  sendTp5Status() 
 ************************************************************************/
void sendTp5Status() {
  static int loopc = 0;
  loopc = ++loopc % 10;
  if (isDumpingData) {
    if ((loopc == 0) || (loopc == 4))  dumpData();
  }
  else if (loopc == 0) sendStatusFrame(XBEE_HC); 
  else if (loopc == 3) sendStatusFrame(XBEE_PC);
}



/************************************************************************
 *  tp5Steer() 
 ************************************************************************/
void tp5Steer(float fps) {
  
//  if (isRouteInProgress) {
//    steerRoute();
//  }
//  else  {
    targetHeading += (controllerX * 2.0);
    fixHeading(tickHeading, targetHeading, fps);
//  }
}


/************************************************************************
 *  fixHeading() adjust the speed to correct for deviation from heading.
 ************************************************************************/
void fixHeading(float hheading, float target, float fps) {
  float speedAdjustment;

  // TODO quickfix using modulo & int, redo later using int/long?
  // Put heading into range of 0-359.99
  long intHeading = (long) (hheading * 100.0) % 36000;
  if (intHeading < 0) intHeading += 36000;
  else if (intHeading > 36000) intHeading -= 36000;
  hheading = ((float) intHeading) / 100.0;

  // Put target into range of 0-359.99
  long intTarget = (long) (target * 100.0) % 36000;
  if (intTarget < 0) intTarget += 36000;
  else if (intTarget > 36000) intTarget -= 36000;
  target = ((float) intTarget) / 100.0;


  float aDiff = target - hheading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  speedAdjustment = constrain(aDiff * 0.02, -1.0, 1.0);
  tp5FpsLeft = tp6FpsLeft = fps + speedAdjustment;
  tp5FpsRight = tp6FpsRight = fps - speedAdjustment;
  magTickCorrection();
}


/************************************************************************
 *  magTickCorrection() Uses wheel ticks.  Called every timed loop (10 millisec.)
 ************************************************************************/
void magTickCorrection() {
  float diff = magHeading - tickHeading;
  if (abs(diff < 360)) {
    //    magCorrection += diff * 0.1;
  }
}

