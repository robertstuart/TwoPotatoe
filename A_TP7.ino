/************************************************************************
 * ----- TP7 ------
 *    Algorithm computations at tick intterrupts to immediately respond to
 *    speed changes (bumps in the road, etc.) and avoid overshoot when
 *    catching up.
 ************************************************************************/

float oldTp7GaPitchTickAngle = 0.0;
float tp7LpfCos = 0.0;
float tp7LpfCosOld = 0.0;
float tp7ControllerSpeed = 0.0;
float tp7AngleError = 0.0;
float tteR = 0.0;
float tteL = 0.0;
long tp7LoopTimeR = 0L;
long tp7LoopTimeL = 0L;

/************************************************************************
 *  aTp7Run() 
 ************************************************************************/
void aTp7Run() {
  txRateDivider = 10;  // Send status to controllers every X imu reads.
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  angleInitTp7();
  motorInitTp7();
  while(mode == MODE_TP7) { // main loop
    readXBee();  // Read commands from PC or Hand Controller
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    flushSerial();
    dumpData();
    checkMotorRight();
    checkMotorLeft();
    if (digitalRead(MPU_INTR_PIN) == HIGH) {
      imu9150.getIntStatus();  // Clear the bit.
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      oldTimeTrigger = timeMicroseconds;
      aTp7(); 
      tp7Log();
//      battery();
      led();
//      safeAngle();
      gravity();
      controllerConnected();
      setTp4RunningState();
      //    checkDrift();
    } // end timed loop
  }
}


/************************************************************************
 *  aTp7() 
 ************************************************************************/
void aTp7() {
  getTp7Angle();
  readSpeed();

  float gaPitchDelta = gaPitch - oldGaPitch;
  oldGaPitch = gaPitch;

  // compute the Center of Oscillation Speed (COS)
  float tp7Cos = wheelSpeedFps + ((*currentValSet).v * gaPitchDelta); // subtract out rotation **************
  tp7LpfCos = tp7LpfCosOld + ((tp7Cos - tp7LpfCosOld) * (*currentValSet).w); // smooth it out a little
  tp7LpfCosOld = tp7LpfCos;

  tp7ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp7SpeedError = tp7ControllerSpeed - tp7LpfCos;

  // compute a weighted angle to eventually correct the speed error
  float tp7TargetAngle = tp7SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  tp7AngleError = gaPitchTickAngle - tp7TargetAngle;

//  tp5Steer();
  setTargetSpeedRight(tp7LpfCos); 
  setTargetSpeedLeft(tp7LpfCos);
  
  // Set the values for the interrupt routines
  tteR = 15.0 * tp5AngleError;  // Target beyond current tickDistance.
  tteL = 15.0 * tp5AngleError;  // Target beyond current tickDistance.
  loopTickDistanceR = tickDistanceRight;
  loopTickDistanceL = tickDistanceLeft;
  tp7LoopTimeR = tickTimeRight;
  tp7LoopTimeL = tickTimeLeft;
  
  // Send status message to controller.
  sendStatusFrame(); 
} // end aTp7() 

/************************************************************************
 *  tp7Log() Put values in the dump arrays.
 ************************************************************************/
void tp7Log() { 
  static int lNum = 0;
  timeArray[dataArrayPtr] = timeMicroseconds;
  tickArray[dataArrayPtr] = tickDistanceRight;
  angleArray[dataArrayPtr] = (long) (gaPitch * 100.0);
  lNum = (lNum+1) % 15;
  motorArray[dataArrayPtr] = 0;
  dataArrayPtr++;
  dataArrayPtr = dataArrayPtr %  DATA_ARRAY_SIZE;
}




