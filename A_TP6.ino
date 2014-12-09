/************************************************************************
 * ----- TP6 ------
 ************************************************************************/
float tp6Fps = 0.0f;
float tp6AngleErrorW = 0.0f;
float tp6ControllerSpeed = 0;
float tp6LoopSec = 0.0f;
float tp6LpfCos = 0.0;
float tp6LpfCosOld = 0.0;
//float targetHeading = 0.0;
float tp6LpfAngleErrorWOld = 0.0;
float tp6LpfAngleErrorW = 0.0;
float tp6AngleError = 0.0;

/************************************************************************
 *  aTp6Run() 
 ************************************************************************/
void aTp6Run() {
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  motorInitTp();
  setBlink(RED_LED_PIN, BLINK_SB);
  while(mode == MODE_TP6) { // main loop
    commonTasks();
//    route();

    // Do the timed loop
    if (readImu()) {
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      oldTimeTrigger = timeMicroseconds;
      tp6LoopSec = ((float) actualLoopTime)/1000000.0;  
      aTp6(); 
      sendTp6Status();
      magTickCorrection();
      safeAngle();
      gravity();
      switches();
      checkMotorRight();
      checkMotorLeft();
//    checkDrift();
    } // end timed loop
  }
}



/************************************************************************
 *  aTp6() 
 ************************************************************************/
void aTp6() {
  readSpeed();
  timeMicroseconds = micros(); // So algorithm will have latest time
  getTp5Angle();  
  float tp6AngleDelta = gyroPitchDelta; //** 2

  // compute the Center of Oscillation Speed (COS)
  float tp6Rotation = ((*currentValSet).v * tp6AngleDelta);
//  float tp6Rotation = 0.7f * tp6AngleDelta;
  float tp6Cos = wheelSpeedFps + tp6Rotation; // subtract out rotation **************
  tp6LpfCos = (tp6LpfCosOld * (1.0 - (*currentValSet).w))  + (tp6Cos  * (*currentValSet).w); // smooth it out a little (0.2)
  tp6LpfCosAccel = tp6LpfCos - tp6LpfCosOld;
  tp6LpfCosOld = tp6LpfCos;

  tp6ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
//  float tp6SpeedError = tp6ControllerSpeed - tp6LpfCos;
  float tp6SpeedError = tp6ControllerSpeed - tp6Cos;

  // compute a weighted angle to eventually correct the speed error
  float tp6TargetAngle = tp6SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  tp6AngleError = gaPitch - tp6TargetAngle;  //** 2

  // Original value for y: 0.09
  tp6AngleErrorW = tp6AngleError * (*currentValSet).y; //******************* Angle error to speed *******************
  tp6LpfAngleErrorW = (tp6LpfAngleErrorWOld * (1.0f - ((*currentValSet).t)))  + (tp6AngleErrorW * (*currentValSet).t);
  tp6LpfAngleErrorWOld = tp6LpfAngleErrorW;

  // Add the angle error to the base speed to get the target speed.
//  tp6Fps = tp6LpfAngleErrorW + tp6LpfCos;
  tp6Fps = tp6AngleErrorW + tp6LpfCos;

  if (!isStateBit(TP_STATE_RUN_AIR)) {
    tp6Steer(tp6Fps);
    setTargetSpeedRight(tp6FpsRight);  // Need to set direction.  Does not set speed!
    setTargetSpeedLeft(tp6FpsLeft);  // Need to set direction.  Does not set speed!
  }
  else {
    setTargetSpeedRight(airFps);
    setTargetSpeedLeft(airFps);
  }
  
addLog((long) (tickPosition),
       (short) (wheelSpeedFps * 100.0),
       (short) (gaPitch * 100.0),
       (short) (tp6LpfCos * 100.0),
       (short) (tp6AngleError * 100.0), 
       (short) (tp6LpfAngleErrorW * 100.0),
       (short) (tp6Fps * 100.0));
 } // end aTp6() 



/************************************************************************
 *  sendTp6Status() 
 ************************************************************************/
void sendTp6Status() {
  static int loopc = 0;
  loopc = ++loopc % 10;
  if (isDumpingData) {
    if ((loopc == 0) || (loopc == 4))  dumpData();
  }
  else if (loopc == 0) sendStatusFrame(XBEE_HC); 
  else if (loopc == 3) sendStatusFrame(XBEE_PC);
}



/************************************************************************
 *  tp6Steer() 
 ************************************************************************/
void tp6Steer(float fps) {
  
//  if (isRouteInProgress) {
//    steerRoute();
//  }
//  else  {
  float speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX;
  tp5FpsLeft = tp6FpsLeft = fps + speedAdjustment;
  tp5FpsRight = tp6FpsRight = fps - speedAdjustment;
//  }
}

