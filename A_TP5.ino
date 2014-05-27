/************************************************************************
 * ----- TP5 -----
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
float oldGaPitchTickAngle = 0.0;
long routeResetTime = 0L;
boolean clockwise = true;
float targetHeading = 0.0;
long targetTickDistance = 0L;
float turnSpeedAdjustment = 0.0;

// Set up IMU
//  compass.init(LSM303DLHC_DEVICE, 0);
//  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
//  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
//  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
//  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth

/************************************************************************
 *  aTp5Run() 
 ************************************************************************/
void aTp5Run() {
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  tp5RawIError = 0.0f;
  motorInitTp();
  while(mode == MODE_TP5) { // main loop
    readXBee();  // Read commands from PC or Hand Controller
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    flushSerial();
    checkMotorRight();
    checkMotorLeft();
    route();

    // Do the timed loop
    if(timeMicroseconds > timeTrigger) {  // Loop executed every XX microseconds 
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      tp5LoopSec = ((float) actualLoopTime)/1000000.0; 
      timeTrigger += 10000; // 100 per second
      oldTimeTrigger = timeMicroseconds;

      aTp5(); 
      battery();
      led();
      safeAngle();
      gravity();
      controllerConnected();
      setTp4RunningState();
      //    checkDrift();
    } // end timed loop
  }
}



/************************************************************************
 *  aTp5() 
 *    
 * 
 * 
 ************************************************************************/
void aTp5() {
  readSpeed();
  getTp5Angle();

  float tp5AngleDelta = gaPitchTickAngle - oldGaPitchTickAngle;
  oldGaPitchTickAngle = gaPitchTickAngle;

  // compute the Center of Oscillation Speed (COS)
  float tp5Cos = wheelSpeedFps + (tp5AngleDelta * 1.4);
  //  float rateCos = wheelSpeedFps + ((*currentValSet).v * gyroXAngleDelta); // subtract out rotation **************
  //  tp5CoSpeed = ((rateCos * (*currentValSet).w)) + ((1.0f - (*currentValSet).w) * tp5OldCospeed); // smooth it out a little
  tp5LpfCos = tp5LpfCosOld + ((tp5Cos - tp5LpfCosOld) * 0.2); // smooth it out a little
  tp5LpfCosOld = tp5LpfCos;

  tp5ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp5SpeedError = tp5ControllerSpeed - tp5LpfCos;

  // compute a weighted angle to eventually correct the speed error
  float tp5TargetAngle = tp5SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  float tp5AngleError = gaPitchTickAngle - tp5TargetAngle;
  tp5AngleErrorW = tp5AngleError * (*currentValSet).y; //******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target speed.
  tp5Fps = tp5AngleErrorW + tp5LpfCos;
//  tp5FpsRight = tp5FpsLeft = tp5Fps;

  tp5Steer();
  setTargetSpeedRight(tp5FpsRight);
  setTargetSpeedLeft(tp5FpsLeft);
  if (!isBlockInProgress) {
    if(txRateHL) txRateDivider = 1;
    else txRateDivider = 5;
  }

  if (isStateBitSet(TP_STATE_STREAMING)) {
    set4Byte(sendArray, TP_SEND_A_VAL, tickDistance);
    set4Byte(sendArray, TP_SEND_B_VAL, targetTickDistance);
    set2Byte(sendArray, TP_SEND_C_VAL, magHeading * 10.0);
    set2Byte(sendArray, TP_SEND_D_VAL, tickHeading * 10.0);
    set2Byte(sendArray, TP_SEND_E_VAL, targetHeading * 10.0);
    set2Byte(sendArray, TP_SEND_F_VAL, wheelSpeedFps * 100.0);
    set2Byte(sendArray, TP_SEND_G_VAL, routeActionPtr);
    sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_H_VAL); 
  } 
  else {
    sendTXFrame(XBEE_BROADCAST, sendArray, 0); 
  }
} // end aTp5() 



/************************************************************************
 *  tp5Steer() 
 ************************************************************************/
void tp5Steer() {
  if (isRouteInProgress) {
    steerRoute();
    return;
  }

  float speedAdjustment = 0.0;
  // Adjust targetHeading by controllerX
  // TODO quickfix using modulo & int, redo later
  targetHeading = targetHeading + (controllerX * 1.0); 
  long intTargetHeading = (long) (targetHeading * 100.0) % 36000;
  if (intTargetHeading < 0) intTargetHeading += 36000;
  else if (intTargetHeading > 36000) intTargetHeading -= 36000;
  targetHeading = ((float) intTargetHeading) / 100.0;
    float aDiff = targetHeading - tickHeading;
    if (aDiff > 180.0) aDiff -= 360.0;
    else if (aDiff < -180.0) aDiff += 360.0;
    
    if (abs(aDiff) < 2.0) speedAdjustment = 0.05;
    else if (abs(aDiff) < 10.0) speedAdjustment = 0.2;
    else speedAdjustment = 0.5;
//speedAdjustment = 0.1;    
    if (aDiff > 0.0) speedAdjustment *= -1.0;
    
    tp5FpsLeft = tp5Fps - speedAdjustment;
    tp5FpsRight = tp5Fps + speedAdjustment;

//Serial.print(speedAdjustment); Serial.print("  ");
//Serial.print(targetHeading); Serial.print("  ");
//Serial.println(tickHeading);
//  if (isRouteInProgress) {
//    steerRoute();
//    return;
//  }
//  float speedAdjustment = 0.0f;
//  boolean controllerZero = abs(controllerX) < 0.05;
//
//  // Adjust straightMode status
//  if (straightMode) {
//    if (!controllerZero) {
//      straightMode = false;  // take it out of straightmode
//    } 
//    else if (isRotating) {
//      speedAdjustment = tp5GetRotation();
//    }
//    else if ((tickDistanceRight - tickDistanceLeft) < tpPositionDiff) {
//      speedAdjustment = -.1;
//    } 
//    else {
//      speedAdjustment = +.1;
//    }
//  }
//  else { // not in straightMode
//    if (controllerZero) {  // Put in StraightMode if controller is zero
//      straightMode = true;  // put it in straightmode
//      tpPositionDiff = tickDistanceRight - tickDistanceLeft;
//      isRotating = false;
//      rotateTarget = 0.0f;
//    }
//  }
//
//  if (!straightMode) {
//    //    speedAdjustment = controllerX * 1.0 - (tp5CoSpeed * 0.1); // factor for turning rate.
//    speedAdjustment = controllerX * 0.6; // factor for turning rate.
//  }
//
//  tp5FpsLeft += speedAdjustment;
//  tp5FpsRight -= speedAdjustment;
}


/************************************************************************
 *  steerRoute() uses wheel ticks
 ************************************************************************/
void steerRoute() {
  float speedAdjustment = 0.0;
  float magDiff = 0.0;
  float aDiff = 0.0;
  
  switch (routeCurrentAction) {
  case 'M': // Mag
    magDiff = magHeading - targetHeading;
    if (magDiff > 180.0) magDiff -= 360.0;
    else if (magDiff < -180.0) magDiff += 360.0;
    if (abs(magDiff) < 2.0) speedAdjustment = 0.1;
    else if (abs(magDiff) < 10.0) speedAdjustment = 0.2;
    else speedAdjustment = 2.0;
    
    if (magDiff > 0.0) speedAdjustment *= -1.0;
// Serial.print(magDiff);
// Serial.print("  ");
// Serial.println(speedAdjustment);   
    
    tp5FpsLeft = tp5Fps + speedAdjustment;
    tp5FpsRight = tp5Fps - speedAdjustment;
    break;
  case 'R':
    routeReset(false);
    // Fall through.
  case 'S': // Speed  // TODO tune this?
    aDiff = targetHeading - tickHeading;
    if (aDiff > 180.0) aDiff -= 360.0;
    else if (aDiff < -180.0) aDiff += 360.0;
    
    if (aDiff > 0.0) {
      tp5FpsLeft = tp5Fps + 0.1;
      tp5FpsRight = tp5Fps - 0.1;  
    } 
    else {
      tp5FpsLeft = tp5Fps - 0.1;
      tp5FpsRight = tp5Fps + 0.1;  
    }
    break;
  case 'T':  // Turn
    tp5FpsLeft = tp5Fps + turnSpeedAdjustment;
    tp5FpsRight = tp5Fps - turnSpeedAdjustment;  
//Serial.print(tp5FpsLeft); Serial.print("  "); Serial.println(tp5FpsRight);
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
      tickDistanceLeft = (long) (deg * TICKS_PER_DEGREE); 
    }
  }
}


float tp5GetRotation() {
  float rotateError = rotateTarget - gyroYawAngle;
  if (abs(rotateError) < 1.0f) {
    isRotating = false;
    tpPositionDiff = tickDistanceRight - tickDistanceLeft;
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
    if (routeResetTime < timeMilliseconds) isNewAction = true;
//Serial.println(timeMilliseconds);
    break;
  case 'M':
    if (abs(magHeading - targetHeading) < 1.0) isNewAction = true;
//Serial.println(magHeading);
    break;
  case 'S':
    if (controllerY > 0.0) {
      if (tickDistance > targetTickDistance) isNewAction = true;
    }
    else {
      if (tickDistance < targetTickDistance) isNewAction = true;
    }
//Serial.println(tickDistance);
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
  
//char cc[] = " ";
//cc[0]= routeCurrentAction;
//Serial.print(cc); 
//Serial.print(routeActionPtr);
//Serial.print("  ");
//Serial.print(aVal);
//Serial.print("  ");
//Serial.print(bVal);
//Serial.println();
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
Serial.print(radius); Serial.print("  "); Serial.println(turnSpeedAdjustment);
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




