/************************************************************************
 * ----- TP6 ------
 ************************************************************************/
float tp6RawIError = 0.0f; 
float tp6ErrorSpeedFps = 0.0f;
float tp6ISpeed = 0.0f;
float tp6Fps = 0.0f;
float tp6CoSpeed = 0.0f;
float tp6OldCospeed = 0.0f; 
float tp6AngleErrorW = 0.0f;
boolean tp6IsHome = false;
float tp6HomeSpeed = 0.0f;
long tp6AccumDistanceError = 0l;
float tp6ControllerSpeed = 0;
float tp6LoopSec = 0.0f;
unsigned int tp6LoopCounter = 0;
float tp6LpfCos = 0.0;
float tp6LpfCosOld = 0.0;
float oldGaPitchAngle6 = 0.0;
float tp6LpfAngleErrorWOld = 0.0;
float tp6LpfAngleErrorW = 0.0;
float tp6AngleError = 0.0;
float tp6LpfCosLeft = 0.0f;
float tp6LpfCosLeftOld = 0.0f;
unsigned long tCheck = 0;

/************************************************************************
 *  aTp6Run() 
 ************************************************************************/
void aTp6Run() {
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  tp6RawIError = 0.0f;
  motorInitTp();
  angleInitTp7();
  while(mode == MODE_TP6) { // main loop
    commonTasks();
//    route();

    // Do the timed loop
    if (digitalRead(MPU_INTR_PIN) == HIGH) {
      imu9150.getIntStatus();  // Clear the bit.
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      oldTimeTrigger = timeMicroseconds;
      tp6LoopSec = ((float) actualLoopTime)/1000000.0; 
      aTp6(); 
      sendTp6Status();
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
 *  aTp6() 
 ************************************************************************/
void aTp6() {
  readSpeed();
  timeMicroseconds = micros(); // So algorithm will have latest time
  getTp6Angle();
tCheck = micros();
//
//  float tp6AngleDelta = gaPitchAngle - oldGaPitchAngle; //** 2
//  oldGaPitchAngle = gaPitchAngle; //** 2
//
//  // compute the Center of Oscillation Speed (COS)
//  float tp6Cos = wheelSpeedFps + ((*currentValSet).v * tp6AngleDelta); // subtract out rotation **************
//  tp6LpfCos = tp6LpfCosOld + ((tp6Cos - tp6LpfCosOld) * (*currentValSet).w); // smooth it out a little (0.2)
//  tp6LpfCosOld = tp6LpfCos;

  // newCos
  int newCos = mWheelSpeedFps + (gyroPitchRaw / 10); // subtract out rotation **************
  newLpfCos = newLpfCosOld + (((newCos - newLpfCosOld) * 20) / 100); // smooth it out a little
  newLpfCosOld = newLpfCos;

  // newCos left
  int newCosLeft = mAverageFpsLeft + (gyroPitchRaw / 10); // subtract out rotation **************
  newLpfCosLeft = newLpfCosLeftOld + (((newCosLeft - newLpfCosLeftOld) * 20) / 100); // smooth it out a little
  newAccelLeft = newLpfCosLeft - newLpfCosLeftOld;
  newLpfCosLeftOld = newLpfCosLeft;

//  // compute the Center of Oscillation Speed (COS) for just the left side
//  float tp6CosLeft = fpsLeft + ((*currentValSet).v * tp6AngleDelta); // subtract out rotation **************
//  tp6LpfCosLeft = tp6LpfCosLeftOld + ((tp6CosLeft - tp6LpfCosLeftOld) * (*currentValSet).w); // smooth it out a little
//  accelLeft = tp6LpfCosLeft - tp6LpfCosLeftOld;
//  tp6LpfCosLeftOld = tp6LpfCosLeft;
//  
  

  tp6ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
//  float tp6SpeedError = tp6ControllerSpeed - tp6LpfCos;
  float tp6SpeedError = tp6ControllerSpeed - (((float) newLpfCos) / 1000.0);

  // compute a weighted angle to eventually correct the speed error
  float tp6TargetAngle = tp6SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  tp6AngleError = gaPitchAngle2 - tp6TargetAngle;  //** 2

  // Original value for y: 0.09
  tp6AngleErrorW = tp6AngleError * (*currentValSet).y; //******************* Angle error to speed *******************
  tp6LpfAngleErrorW = tp6LpfAngleErrorWOld + ((tp6AngleErrorW - tp6LpfAngleErrorWOld) * 0.1);
  tp6LpfAngleErrorWOld = tp6LpfAngleErrorW;

  // Add the angle error to the base speed to get the target speed.
//  tp6Fps = tp6LpfAngleErrorW + tp6LpfCos;
  tp6Fps = tp6LpfAngleErrorW + (((float) newLpfCos) / 1000.0);
//  if (damp()) {
//    tp6Fps = tp6LpfCos;
//  }

  tp6Steer(tp6Fps);
  setTargetSpeedRight(tp6FpsRight);
  setTargetSpeedLeft(tp6FpsLeft);
runLog(timeMicroseconds, (int) (tp6Fps * 1000.0), (int)  (gaPitchAngle2 * 1000.0), 00);
//Serial.println(gaPitchAngle);
} // end aTp6() 



/************************************************************************
 *  sendTp6Status() 
 ************************************************************************/
void sendTp6Status() {
  static int loopc = 0;
  loopc = ++loopc % 10;
  if (isDumpingData) {
    dumpData();
  }
  else if (loopc == 0) sendStatusFrame(XBEE_HC); 
  else if (loopc == 3) sendStatusFrame(XBEE_PC);
}



/************************************************************************
 *  tp6Steer() 
 ************************************************************************/
void tp6Steer(float fps) {
    targetHeading += (controllerX * 2.0);
    fixHeading(tickHeading, targetHeading, fps);
}

