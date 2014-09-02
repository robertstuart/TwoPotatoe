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
long mLpfCos;
unsigned long sTime, sStartTime;

/************************************************************************
 *  aTp7Run() 
 ************************************************************************/
void aTp7Run() {
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
    checkTp7MotorRight();
    checkTp7MotorLeft();
    if (digitalRead(MPU_INTR_PIN) == HIGH) {
      imu9150.getIntStatus();  // Clear the bit.
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      oldTimeTrigger = timeMicroseconds;
      led();
      aTp7(); 
      battery();
      gravity();
      controllerConnected();
      setRunningState();
      sendStatusFrame();  // Send status message to controller.
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
  mLpfCos = (long) (tp7LpfCos * 1000.0);
  
//  long coPosition = tickDistance + ((long) (gaPitch * TICKS_PER_PITCH_DEGREE));


  tp7ControllerSpeed = controllerY * SPEED_MULTIPLIER; //+-3.0 fps

  // find the speed error
  float tp7SpeedError = tp7ControllerSpeed - tp7LpfCos;

  // Compute a target angle that is proportional to the speed error.
  float tp7TargetAngle = tp7SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute a target wheel position for this angle.
  int tickError = ((int) ((gaPitch - tp7TargetAngle) * ((float) TICKS_PER_PITCH_DEGREE))) / 2;
//  long targetWheelPosition = tickDistance + tickError;
//  tp5Steer();

  // Set the target tick period to the COS
//  setTargetSpeedRight(tp7LpfCos); 
//  setTargetSpeedLeft(tp7LpfCos);

  // Set the values for the isr 
  baseTickTimeRight = tickTimeRight;
  baseTargetTickDistanceRight = tickDistanceRight + tickError;
  baseTickTimeLeft = tickTimeLeft;
  baseTargetTickDistanceLeft = tickDistanceLeft + tickError;
  cosTickPeriodRight = cosTickPeriodLeft = (long) (ENC_FACTOR / tp7LpfCos);
  int mTp7LpfCos = (int) (tp7LpfCos * 1000.0);
  
  motorTp6Right(tickError);
  motorTp6Left(tickError);

//////////////////////////////////////////// 
//if ((serCount++ % 50) == 1) {           //
//  Serial.print(debugValA);              //
//  Serial.print("\t");                   //
//  Serial.print(debugValB);              //
//  Serial.print("  ");                   //
//  Serial.print(debugValC);              //
//  Serial.print("  ");                   //
//  Serial.print(debugValD);              //
//  Serial.print("  ");                   //
//  Serial.println(debugValE);            //
//}                                       //
////////////////////////////////////////////

} // end aTp7() 

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




