/***********************************************************************.
 * ----- TP6 ------
 ***********************************************************************/
float tp6Fps = 0.0f;
float fpsCorrection = 0.0f;
float tp6ControllerSpeed = 0;
//float tp6LoopSec = 0.0f;
float tp6LpfCos = 0.0;
float tp6LpfCosOld = 0.0;
//float targetHeading = 0.0;
float fpsLpfCorrectionOld = 0.0;
float fpsLpfCorrection = 0.0;
float tp6AngleError = 0.0;
float tp6TargetAngle = 0.0;
float tp6Cos = 0.0; 
float tp6Rotation = 0.0;

/***********************************************************************.
 *  aTp6Run() 
 ***********************************************************************/
void aTp6Run() {
  unsigned int accelCycle = 0;
  
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  angleInit6();
//  motorInitTp6();
  motorInitTp();
  currentValSet = &tp6;
  setBlink(RED_LED_PIN, BLINK_SB);
  while(mode == MODE_TP6) { // main loop
    commonTasks();
//    route();
//    readAccel();
    // Do the timed loop
    timeMicroseconds = micros();
    if (timeMicroseconds > timeTrigger) {
      readGyro();
      if ((++accelCycle % 4)  == 1) readAccel();  // Read every 4th time.
      aTp6(); 
      sendTp6Status();
      magTickCorrection();
      safeAngle();
      switches();
//      checkMotor6Right();
//      checkMotor6Left();
      checkMotorRight();
      checkMotorLeft();
//    checkDrift();
//      log6();
      timeTrigger +=  2500; // 400/sec
    } // end timed loop
  }
}



/***********************************************************************.
 *  aTp6() 
 ***********************************************************************/
void aTp6() {
//  getAngle();
  readSpeed();
  // compute the Center of Oscillation Speed (COS)
  tp6Rotation = (*currentValSet).t * gyroPitchDelta; // 3.6
  tp6Cos = wheelSpeedFps + tp6Rotation; // subtract rotation 
  tp5LpfCos = tp6LpfCos = (tp6LpfCosOld * (1.0 - (*currentValSet).u))  + (tp6Cos  * (*currentValSet).u); // smooth it out a little (0.2)
//  tp6LpfCos = (tp6LpfCosOld * (1.0 - 0.05)) + (tp6Cos * 0.05); // smooth it out a little (0.2)
  tp6LpfCosAccel = tp6LpfCos - tp6LpfCosOld;
  tp6LpfCosOld = tp6LpfCos;

  tp6ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp6SpeedError = tp6ControllerSpeed - tp6LpfCos;

  // compute a weighted angle to eventually correct the speed error
  tp6TargetAngle = tp6SpeedError * (*currentValSet).v; //************ Speed error to angle *******************
//  float tp6TargetAngle = tp6SpeedError * 2.0; //********** Speed error to angle *******
  
  // Compute angle error and weight factor
  tp6AngleError = gaPitch - tp6TargetAngle;  //** 2
  fpsCorrection = tp6AngleError * (*currentValSet).w; //******************* Angle error to speed *******************
//  speedCorrection = tp6AngleError * 0.18; //******************* Angle error to speed *******************
//  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - 0.1))  + (speedCorrection * 0.1);
  fpsLpfCorrection = (fpsLpfCorrectionOld * (1.0f - ((*currentValSet).x)))  + (fpsCorrection * ((*currentValSet).x));
  fpsLpfCorrectionOld = fpsLpfCorrection;

  // Add the angle error to the base speed to get the target speed.
  tp6Fps = fpsLpfCorrection + tp6LpfCos;
  tp6Steer(tp6Fps);
  targetMFpsRight = tp6FpsRight * 1000; // 
  targetMFpsLeft = tp6FpsLeft * 1000; // 

} // end aTp6() 



/***********************************************************************.
 *  sendTp6Status() 
 ***********************************************************************/
void sendTp6Status() {
  static unsigned int loopc = 0;
  loopc = ++loopc % 40;
  if (isDumpingData) {
    if ((loopc == 0) || (loopc == 4))  dumpData();
  }
  else if (loopc == 0) sendStatusFrame(XBEE_HC); 
  else if (loopc == 10) {
	sendStatusFrame(XBEE_PC);
  }
  else if (loopc == 20) { // debugging print statements
//    Serial.print(gyroPitch);
//    Serial.print("\t");
//    Serial.print(accelPitchAngle);
//    Serial.print("\t");
//    Serial.println();
  }
  else if (loopc == 30) {
    readSonar();
  }
  addLog(
          (long) (timeMicroseconds),
          (short) (tp6Rotation * 100.0),
          (short) (gaPitch * 100.0),
          (short) (accelPitch * 100.0),
          (short) (gyroPitch * 100.0),
          (short) (tp6TargetAngle * 100.0),
          (short) (fpsLpfCorrection * 100.0));
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
void tp6Steer(float fps) {
  
//  if (isRouteInProgress) {
//    steerRoute();
//  }
//  else  {
  float speedAdjustment = (((1.0 - abs(controllerY)) * 1.5) + 0.5) * controllerX;
//      float speedAdjustment = controllerX * 1.0;
  tp6FpsLeft = fps + speedAdjustment;
  tp6FpsRight = fps - speedAdjustment;
//    targetHeading += (controllerX * 2.0);
//    fixHeading(tickHeading, targetHeading, fps);
//  }
}

