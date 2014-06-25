/************************************************************************
 * ----- TP6 -----
 ************************************************************************/
float tp6FpsLeft = 0.0f;
float tp6FpsRight = 0.0f;
float tp6RawIError = 0.0f; 
float tp6ErrorSpeedFps = 0.0f;
float tp6ISpeed = 0.0f;
float tp6Fps = 0.0f;
float tp6CoSpeed = 0.0f;
float tp6OldCospeed = 0.0f; 
float tp6AngleErrorW = 0.0f;
boolean tp6IsHome = false;
//long tp6HomeDistance = 0L;
float tp6HomeSpeed = 0.0f;
long tp6AccumDistanceError = 0l;
float tp6ControllerSpeed = 0;
float tp6LoopSec = 0.0f;
unsigned int tp6LoopCounter = 0;
float tp6LpfCos = 0.0;
float tp6LpfCosOld = 0.0;
long tp6RouteResetTime = 0L;
boolean tp6Route = true;
float tp6TargetHeading = 0.0;
long tp6TargetTickDistance = 0L;
float tp6TurnSpeedAdjustment = 0.0;
float tp6LpfAngleErrorWOld = 0.0;
float tp6LpfAngleErrorW = 0.0;

// Set up IMU
//  compass.init(LSM303DLHC_DEVICE, 0);
//  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
//  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
//  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
//  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth

/************************************************************************
 *  atp6Run() 
 ************************************************************************/
void aTp6Run() {
  txRateDivider = 5;  // 20/sec
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  m6MotorInitTp();
  while(mode == MODE_TP6) { // main loop
    readXBee();  // Read commands from PC or Hand Controller
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    flushSerial();
    checkMotorRight();
    checkMotorLeft();
    tp6RunRoute();

    // Do the timed loop
    if(timeMicroseconds > timeTrigger) {  // Loop executed every XX microseconds 
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      tp6LoopSec = ((float) actualLoopTime)/1000000.0; 
      timeTrigger += 10000; // 100 per second
      oldTimeTrigger = timeMicroseconds;

      atp6(); 
      tp6MagTickCorrection();
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
 *  atp6() 
 *    
 * 
 * 
 ************************************************************************/
void atp6() {
  readSpeed();
  getTp5Angle();

  float tp6AngleDelta = gaPitchAngle - oldGaPitchTickAngle;
  oldGaPitchTickAngle = gaPitchAngle;

  // compute the Center of Oscillation Speed (COS)
  float tp6Cos = wheelSpeedFps + (tp6AngleDelta * 1.4);
  tp6LpfCos = tp6LpfCosOld + ((tp6Cos - tp6LpfCosOld) * 0.2); // smooth it out a little
  tp6LpfCosOld = tp6LpfCos;

  tp6ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp6SpeedError = tp6ControllerSpeed - tp6LpfCos;

  // compute a weighted angle to eventually correct the speed error
  float tp6TargetAngle = tp6SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  float tp6AngleError = gaPitchTickAngle - tp6TargetAngle;

  // Original value for y: 0.09
  tp6AngleErrorW = tp6AngleError * 0.18; //******************* Angle error to speed *******************
  tp6LpfAngleErrorW = tp6LpfAngleErrorWOld + ((tp6AngleErrorW - tp6LpfAngleErrorWOld) * 0.1);
  tp6LpfAngleErrorWOld = tp6LpfAngleErrorW;


  // Add the angle error to the base speed to get the target speed.
  tp6Fps = tp6LpfAngleErrorW + tp6LpfCos;

  tp6Steer();
  if (tp6FpsRight >= 0.0) targetDirectionRight = FWD;
  else targetDirectionRight = BKWD;
  if (tp6FpsLeft >= 0.0) targetDirectionLeft = FWD;
  else targetDirectionLeft = BKWD;
  
  // Set the values for the interrupt routines
  targetTDR = tickDistanceRight + (15.0 * tp6AngleError);  // Target beyond current tickDistance.
  targetTDL = tickDistanceLeft + (15.0 * tp6AngleError);  // Target beyond current tickDistance.
  fpsRightLong = (long) (tp6FpsRight * 100.0);
  fpsLeftLong = (long) (tp6FpsLeft * 100.0);
  cosLong = (long) (tp6LpfCos * 100.0);
  loopTickDistanceR = tickDistanceRight;
  loopTickDistanceL = tickDistanceLeft;
  tp6LoopTimeR = tickTimeRight;
  tp6LoopTimeL = tickTimeLeft;
//  long ttdR = (10000  * TICKS_PER_TFOOT * fpsRightLong) / 10000000L; // ticks per period @ 1 fps.
  
  if (!isBlockInProgress) {
    if(txRateHL) txRateDivider = 1;
    else txRateDivider = 5;
  }

  if (isStateBitSet(TP_STATE_DATA)) {
    set4Byte(sendArray, TP_SEND_A_VAL, timeMilliseconds);
    set4Byte(sendArray, TP_SEND_B_VAL, tickDistanceRight);
    set2Byte(sendArray, TP_SEND_C_VAL, wheelSpeedFps * 100.0);
    set2Byte(sendArray, TP_SEND_D_VAL, gaPitchAngle * 100.0);
    set2Byte(sendArray, TP_SEND_E_VAL, tp6TargetAngle * 100.0);
    set2Byte(sendArray, TP_SEND_F_VAL, tp6AngleError * 100.0);
    set2Byte(sendArray, TP_SEND_G_VAL, tp6Fps * 100.0);
    
//    set4Byte(sendArray, TP_SEND_A_VAL, debugA);
//    set4Byte(sendArray, TP_SEND_B_VAL, debugB);
//    set2Byte(sendArray, TP_SEND_C_VAL, debugC);
//    set2Byte(sendArray, TP_SEND_D_VAL, magHeading * 10.0);
//    set2Byte(sendArray, TP_SEND_E_VAL, tickHeading * 10.0);
//    set2Byte(sendArray, TP_SEND_F_VAL, tp6TargetHeading * 10.0);
//    set2Byte(sendArray, TP_SEND_G_VAL, routeActionPtr);
    
//    set4Byte(sendArray, TP_SEND_A_VAL, debugA);
//    set4Byte(sendArray, TP_SEND_B_VAL, debugB);
//    set2Byte(sendArray, TP_SEND_C_VAL, debugC);
//    set2Byte(sendArray, TP_SEND_D_VAL, debugD);
//    set2Byte(sendArray, TP_SEND_E_VAL, debugE);
//    set2Byte(sendArray, TP_SEND_F_VAL, debugF);
//    set2Byte(sendArray, TP_SEND_G_VAL, debugG);
    
    sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_H_VAL); 
  } 
  else {
    sendTXFrame(XBEE_BROADCAST, sendArray, 0); 
  }
} // end atp6() 



/************************************************************************
 *  tp6Steer() 
 ************************************************************************/
void tp6Steer() {
  if (isRouteInProgress) {
    tp6SteerRoute();
  }
  else  {
    tp6TargetHeading += (controllerX * 2.0);
    tp6FixHeading(tickHeading, tp6TargetHeading);
  }

}


/************************************************************************
 *  tp6FixHeading() adjust the speed to correct for deviation from heading.
 ************************************************************************/
void tp6FixHeading(float hheading, float target) {
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
  tp6FpsLeft = tp6Fps + speedAdjustment;
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6MagTickCorrection();
}


/************************************************************************
 *  tp6MagTickCorrection() Uses wheel ticks.  Called every timed loop (10 millisec.)
 ************************************************************************/
void tp6MagTickCorrection() {
  float diff = magHeading - tickHeading;
  if (abs(diff < 360)) {
    //    magCorrection += diff * 0.1;
  }
}

/************************************************************************
 *  tp6SteerRoute() Uses wheel ticks.  Called every timed loop (10 millisec.)
 ************************************************************************/
void tp6SteerRoute() {
  float speedAdjustment = 0.0;
  float magDiff = 0.0;
  float aDiff = 0.0;

  switch (routeCurrentAction) {
  case 'M': // Mag
    tp6FixHeading(magHeading, tp6TargetHeading);
    tickDistanceRight = 0;
    tickDistanceLeft = (long) (magHeading * TICKS_PER_DEGREE); 
    break;
  case 'R':
    tp6RouteReset(false);
    tp6FixHeading(tickHeading, tp6TargetHeading);
    // Fall through.
  case 'S': // Speed  // TODO tune this?
    aDiff = tp6TargetHeading - tickHeading;
    tp6FixHeading(tickHeading, tp6TargetHeading);
    break;
  case 'T':  // Turn
    tp6FpsLeft = tp6Fps + tp6TurnSpeedAdjustment;
    tp6FpsRight = tp6Fps - tp6TurnSpeedAdjustment;  
    break;
  case 'Z':  // Zero speed at current bearing.
    tp6FixHeading(tickHeading, tp6TargetHeading); 
    break; 
  }  
}


/************************************************************************
 *  tp6RouteReset() do it here rather than in switch just to break it out.
 *               Averages Mag readings for 6 seconds before start button 
 *               is pressed until 1 second before button is pressed.
 *               We do calculations in timed loop to get consistent average.
 ************************************************************************/
void tp6RouteReset(boolean r) { 
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
  magCorrection = 0.0;
}


float tp6GetRotation() {
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
void tp6RunRoute() {
  float tbd;
  if (!isRouteInProgress) return;
  boolean isNewAction = false;
  switch (routeCurrentAction) {
  case 'R':
    if (digitalRead(YE_SW_PIN) == LOW) isNewAction = true;
    if (digitalRead(RIGHT_HL_PIN) == HIGH) isNewAction = true;
    if (tp6RouteResetTime < timeMilliseconds) isNewAction = true;
    break;
  case 'M':
    if (abs(magHeading - tp6TargetHeading) < 1.0) isNewAction = true;
    break;
  case 'S':
    if (controllerY > 0.0) {
      if (tickDistance > tp6TargetTickDistance) isNewAction = true;
    }
    else {
      if (tickDistance < tp6TargetTickDistance) isNewAction = true;
    }
    break;
  case 'T':
    tbd =  (tp6TargetHeading - tickHeading);
    if (tbd < 180) tbd += 360.0;
    if (tbd > 180) tbd -= 360.0;
    if (tp6Route) {
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
    tp6SetNewRouteAction();
  }
}



/************************************************************************
 *  setNewRouteAction() Start the next action in the route
 ************************************************************************/
void tp6SetNewRouteAction() {
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
    tp6TargetHeading = ((float) aVal) / 10.0;
    break;
  case 'R':   // Reset
    controllerY = 0.0;
    tp6RouteResetTime = (aVal * 100); // assume seconds are * 10
    if (tp6RouteResetTime == 0) tp6RouteResetTime = 1000000L; // zero means wait
    if (tp6RouteResetTime < 7000) tp6RouteResetTime = 7000; // need at least 7 sec.
    tp6RouteResetTime += timeMilliseconds;
    tp6RouteReset(true); // get it started.
    break;
  case 'S': // Speed
    controllerY = (((float) aVal) / 100.0) / SPEED_MULTIPLIER;
    tp6TargetTickDistance = tickDistance + (bVal * 10);
    break;
  case 'T': // Turn
    // Degrees go from 0 to 369.9 in .1 increments (unsigned int).
    tp6TargetHeading = ((float) aVal) / 10.0;
    tbd =  (tp6TargetHeading - tickHeading);
    if (tbd < 180) tbd += 360.0;
    if (tbd > 180) tbd -= 360.0;
    if (tbd < 0.0) tp6Route = true;
    else tp6Route = false;
    // radius calculations
    s = controllerY * SPEED_MULTIPLIER;
    radius = ((float) bVal) * 0.2;
    tp6TurnSpeedAdjustment = s/radius;
    if (tp6Route) tp6TurnSpeedAdjustment = tp6TurnSpeedAdjustment * -1.0;
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





