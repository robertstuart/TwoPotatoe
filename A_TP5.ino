/************************************************************************
 * ----- TP5 ------
 ************************************************************************/
float tp5Fps = 0.0f;
float tp5AngleErrorW = 0.0f;
float tp5ControllerSpeed = 0;
float tp5LoopSec = 0.0f;
float tp5LpfCos = 0.0;
float tp5LpfCosOld = 0.0;
float targetHeading = 0.0;
float tp5LpfAngleErrorWOld = 0.0;
float tp5LpfAngleErrorW = 0.0;
float tp5AngleError = 0.0;

/************************************************************************
 *  aTp5Run() 
 ************************************************************************/
void aTp5Run() {
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  angleInit();
  motorInitTp();
  currentValSet = &tp4A;
  setBlink(RED_LED_PIN, BLINK_SB);
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
      switches();
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
  getAngle();
  readSpeed();
  // compute the Center of Oscillation Speed (COS)
  float tp5Rotation = 1.4 * gyroPitchDelta;
  float tp5Cos = wheelSpeedFps + tp5Rotation; // subtract rotation 
//  tp5LpfCos = (tp5LpfCosOld * (1.0 - ((*currentValSet).w))) + (tp5Cos * (*currentValSet).w); // smooth it out a little (0.2)
  tp5LpfCos = (tp5LpfCosOld * (1.0 - 0.2)) + (tp5Cos * 0.2); // smooth it out a little (0.2)
  tp5LpfCosAccel = tp5LpfCos - tp5LpfCosOld;
  tp5LpfCosOld = tp5LpfCos;

  tp5ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp5SpeedError = tp5ControllerSpeed - tp5LpfCos;

  // compute a weighted angle to eventually correct the speed error
//  float tp5TargetAngle = tp5SpeedError * (*currentValSet).x; //************ Speed error to angle *******************
  float tp5TargetAngle = tp5SpeedError * 2.0; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  tp5AngleError = gaPitch - tp5TargetAngle;  //** 2

  // Original value for y: 0.09
//  tp5AngleErrorW = tp5AngleError * (*currentValSet).y; //******************* Angle error to speed *******************
  tp5AngleErrorW = tp5AngleError * 0.18; //******************* Angle error to speed *******************
  tp5LpfAngleErrorW = (tp5LpfAngleErrorWOld * (1.0 - 0.1)) + (tp5AngleErrorW * 0.1);
  tp5LpfAngleErrorWOld = tp5LpfAngleErrorW;

  // Add the angle error to the base speed to get the target speed.
  tp5Fps = tp5LpfAngleErrorW + tp5LpfCos;
  if (!isStateBit(TP_STATE_RUN_AIR)) {
    tp5Steer(tp5Fps);
    setTargetSpeedRight(tp5FpsRight);
    setTargetSpeedLeft(tp5FpsLeft);
  }
  else {
    setTargetSpeedRight(airFps);
    setTargetSpeedLeft(airFps);
  }

//addLog((long) (tpState),
//       (short) (tickPositionRight),
//       (short) (tickPositionLeft),
//       (short) (tickHeading * 10.0),
//       (short) (targetHeading * 10.0), 
//       (short) (tp5FpsRight * 100.0),
//       (short) (tp5FpsLeft * 100.0));
  
} // end aTp5() 



/************************************************************************
 *  sendTp5Status() 
 ************************************************************************/
void sendTp5Status() {
  static unsigned int loopc = 0;
  loopc = ++loopc % 10;
  if (isDumpingData) {
    if ((loopc == 0) || (loopc == 4))  dumpData();
  }
  else if (loopc == 0) {
    sendStatusFrame(XBEE_HC); 
  }
//  else if (loopc == 1) {
//  sendStatusFrame(BLUETOOTH); 
//}
  else if (loopc == 2) {
    sendStatusFrame(XBEE_PC);
  }
  else if (loopc == 4) { // print some debug info
    int a = digitalRead(BU_SW_PIN); // Blue switch
    int b = digitalRead(YE_SW_PIN); // 
    int c = digitalRead(RE_SW_PIN); // 
    int d = digitalRead(GN_SW_PIN); // 
    Serial.print(a); Serial.print("\t");
    Serial.print(b); Serial.print("\t");
    Serial.print(c); Serial.print("\t");
    Serial.print(d); Serial.println();
  }
}



/************************************************************************
 *  tp5Steer() 
 ************************************************************************/
void tp5Steer(float fps) {
  
//  if (isRouteInProgress) {
//    steerRoute();
//  }
//  else  {
      float speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX;
//      float speedAdjustment = controllerX * 1.0;
  tp5FpsLeft = fps + speedAdjustment;
  tp5FpsRight = fps - speedAdjustment;
//    targetHeading += (controllerX * 2.0);
//    fixHeading(tickHeading, targetHeading, fps);
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


  aDiff = target - hheading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  speedAdjustment = constrain(aDiff * 0.02, -1.0, 1.0);
  tp5FpsLeft = tp6FpsLeft = fps + speedAdjustment;
  tp5FpsRight = tp6FpsRight = fps - speedAdjustment;
  magTickCorrection();
  
  addLog((long) (tpState),
       (short) (tickHeading * 10.0),
       (short) (targetHeading * 10.0),
       (short) (hheading * 10.0),
       (short) (target * 10.0), 
       (short) (speedAdjustment * 100.0),
       (short) (tp5FpsLeft * 100.0));

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

