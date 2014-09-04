/************************************************************************
 * ----- TP5 ------
 *    Algorithm using the MC5 motor controller
 * 
 * 
 ************************************************************************/
float tp5FpsLeft = 0.0f;
float tp5FpsRight = 0.0f;
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
  angleInitTp7();
  while(mode == MODE_TP5) { // main loop
    commonTasks();
//    route();

    // Do the timed loop
    if (digitalRead(MPU_INTR_PIN) == HIGH) {
      imu9150.getIntStatus();  // Clear the bit.
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      oldTimeTrigger = timeMicroseconds;
      tp5LoopSec = ((float) actualLoopTime)/1000000.0; 
      aTp5(); 
      sendTp5Status();
      magTickCorrection();
      safeAngle();
      gravity();
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

  float tp5AngleDelta = gaPitchAngle - oldGaPitchAngle; //** 2
  oldGaPitchAngle = gaPitchAngle; //** 2

  // compute the Center of Oscillation Speed (COS)
  float tp5Cos = wheelSpeedFps + ((*currentValSet).v * tp5AngleDelta); // subtract out rotation **************
  tp5LpfCos = tp5LpfCosOld + ((tp5Cos - tp5LpfCosOld) * (*currentValSet).w); // smooth it out a little (0.2)
  tp5LpfCosOld = tp5LpfCos;

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

  tp5Steer();
  setTargetSpeedRight(tp5FpsRight);  // Need to set direction.  Does not set speed!
  setTargetSpeedLeft(tp5FpsLeft);  // Need to set direction.  Does not set speed!
tp7Log((long) (gaPitchAngle *1000.0), (long) (tp5TargetAngle * 1000.0) , (long) (tp5AngleErrorW * 1000.0), (long) (tp5FpsLeft * 1000.0f));
Serial.println(gaPitchAngle);
//if (aRoll == 0) aRoll = 1;
//tp7Log((long) (gaPitchAngle * 1000.0),  (long) (tp5AngleError * 1000.0), (long) (tp5AngleErrorW * 1000.0),  (long) (tp5Fps * 1000.0));
  
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
    dumpData();
  }
  else if (loopc == 0) sendStatusFrame(XBEE_HC); 
  else if (loopc == 3) sendStatusFrame(XBEE_PC);
}



/************************************************************************
 *  tp5Steer() 
 ************************************************************************/
void tp5Steer() {
  
//  if (isRouteInProgress) {
//    steerRoute();
//  }
//  else  {
    targetHeading += (controllerX * 2.0);
    fixHeading(tickHeading, targetHeading);
//  }
}


/************************************************************************
 *  fixHeading() adjust the speed to correct for deviation from heading.
 ************************************************************************/
void fixHeading(float hheading, float target) {
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
  tp5FpsLeft = tp5Fps + speedAdjustment;
  tp5FpsRight = tp5Fps - speedAdjustment;
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

/************************************************************************
 *  steerRoute() Uses wheel ticks.  Called every timed loop (10 millisec.)
 ************************************************************************/
void steerRoute() {
  float speedAdjustment = 0.0;
  float magDiff = 0.0;
  float aDiff = 0.0;

  switch (routeCurrentAction) {
  case 'M': // Mag
    fixHeading(magHeading, targetHeading);
    tickDistanceRight = 0;
    tickDistanceLeft = (long) (magHeading * TICKS_PER_YAW_DEGREE); 
    break;
  case 'R':
    routeReset(false);
    fixHeading(tickHeading, targetHeading);
    // Fall through.
  case 'S': // Speed  // TODO tune this?
    aDiff = targetHeading - tickHeading;
    fixHeading(tickHeading, targetHeading);
    break;
  case 'T':  // Turn
    tp5FpsLeft = tp5Fps + turnSpeedAdjustment;
    tp5FpsRight = tp5Fps - turnSpeedAdjustment;  
    break;
  case 'Z':  // Zero speed at current bearing.
    fixHeading(tickHeading, targetHeading); 
    break; 
  }  
}


/************************************************************************
 *  routeReset() do it here rather than in switch just to break it out.
 *               Averages Mag readings for 6 seconds before start button 
 *               is pressed until 1 second before button is pressed.
 *               We do calculations in timed loop to get consistent average.
 ************************************************************************/
void routeReset(boolean r) { 
  static const int MCOUNT_MAX = 500;
  static float sum;
  static int mCount = 0;
  static int wait = 0;

  if (r == true) { // Start from beginning?
    mCount = 0;
    sum = 0;
    wait = 0;
  }
  if ((wait++ > 100) && (mCount < MCOUNT_MAX)) {  
    sum += magHeading;
    mCount++;
    if (mCount >= MCOUNT_MAX) {
      float deg = sum / ((float) MCOUNT_MAX);
      tickDistanceRight = 0;
      tickDistanceLeft = (long) (deg * TICKS_PER_YAW_DEGREE); 
    }
  }
  magCorrection = 0.0;
}


float tp5GetRotation() {
  float rotateError = rotateTarget - gyroYawAngle;
  if (abs(rotateError) < 1.0f) {
    isRotating = false;
    tpDistanceDiff = tickDistanceRight - tickDistanceLeft;
  }
  float rotationRate = rotateError * 0.1f;
  rotationRate = constrain(rotationRate, -2.0f, 2.0f);
  return rotationRate;
}

/************************************************************************
 *  Route() Check if target is reached and need to move to next action.
 ************************************************************************/
void route() {
  float tbd;
  if (!isRouteInProgress) return;
  boolean isNewAction = false;
  switch (routeCurrentAction) {
  case 'R':
    if (digitalRead(YE_SW_PIN) == LOW) isNewAction = true;
    if (digitalRead(RIGHT_HL_PIN) == HIGH) isNewAction = true;
    if (routeResetTime < timeMilliseconds) isNewAction = true;
    break;
  case 'M':
    if (abs(magHeading - targetHeading) < 1.0) isNewAction = true;
    break;
  case 'S':
    if (controllerY > 0.0) {
      if (tickDistance > targetTickDistance) isNewAction = true;
    }
    else {
      if (tickDistance < targetTickDistance) isNewAction = true;
    }
    break;
  case 'T':
    tbd =  (targetHeading - tickHeading);
    if (tbd < 180) tbd += 360.0;
    if (tbd > 180) tbd -= 360.0;
    if (clockwise) {
      if (tbd > 0) isNewAction = true;
    }
    else {
      if (tbd < 0) isNewAction = true;
    }
    break;
  case 'Z':
    if (timeMilliseconds > routeTargetTime) isNewAction = true;
    break;
  } // end switch()

  // Move to new action if current action is done.
  if (isNewAction) { // Move to next action in list.  
    setNewRouteAction();
  }
}



/************************************************************************
 *  setNewRouteAction() Start the next action in the route
 ************************************************************************/
void setNewRouteAction() {
  static float k = 2; // Adjust k to get one wheel stopped with radius of 0.5
  float s, sa, radius, tbd;
  float motA, motB;

  if (routeActionPtr >= routeActionSize) {
    isRouteInProgress = false;
    return;
  }
  routeCurrentAction = actionArray[routeActionPtr];
  int aVal = aValArray[routeActionPtr];
  int bVal = bValArray[routeActionPtr];

  switch (routeCurrentAction) {
  case 'M': // Mag Heading
    targetHeading = ((float) aVal) / 10.0;
    break;
  case 'R':   // Reset
    controllerY = 0.0;
    routeResetTime = (aVal * 100); // assume seconds are * 10
    if (routeResetTime == 0) routeResetTime = 1000000L; // zero means wait
    if (routeResetTime < 7000) routeResetTime = 7000; // need at least 7 sec.
    routeResetTime += timeMilliseconds;
    routeReset(true); // get it started.
    break;
  case 'S': // Speed
    controllerY = (((float) aVal) / 100.0) / SPEED_MULTIPLIER;
    targetTickDistance = tickDistance + (bVal * 10);
    break;
  case 'T': // Turn
    // Degrees go from 0 to 369.9 in .1 increments (unsigned int).
    targetHeading = ((float) aVal) / 10.0;
    tbd =  (targetHeading - tickHeading);
    if (tbd < 180) tbd += 360.0;
    if (tbd > 180) tbd -= 360.0;
    if (tbd < 0.0) clockwise = true;
    else clockwise = false;
    // radius calculations
    s = controllerY * SPEED_MULTIPLIER;
    radius = ((float) bVal) * 0.2;
    turnSpeedAdjustment = s/radius;
    if (clockwise) turnSpeedAdjustment = turnSpeedAdjustment * -1.0;
    break;
  case 'Z':
    controllerY = 0.0;
    routeTargetTime = (aValArray[routeActionPtr] * 10) + timeMilliseconds;
    break;
  }

  if (routeActionPtr == 0) { // First call in a route.
    tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  }
  routeActionPtr++;
}



/************************************************************************
 *  tp7Log() Put values in the dump arrays.
 ************************************************************************/
void tp7Log(long aVal, long bVal, long cVal, long dVal) { 
  if (!isDumpingData) {
    if (aVal == 0L) aVal = 1L; // don't indicate end
    aArray[dataArrayPtr] = aVal;
    bArray[dataArrayPtr] = bVal;
    cArray[dataArrayPtr] = cVal;
    dArray[dataArrayPtr] = dVal;
    dataArrayPtr++;
    dataArrayPtr = dataArrayPtr %  DATA_ARRAY_SIZE;
  }
}




