/************************************************************************
 * ----- TP4 -----
 *    Algorithm using the TP motor controller
 * 
 * 
 ************************************************************************/
unsigned long lasttime, gap;


float tp4FpsLeft = 0.0f;
float tp4FpsRight = 0.0f;
float tp4RawIError = 0.0f; 
float tp4ErrorSpeedFps = 0.0f;
float tp4ISpeed = 0.0f;
float tp4Fps = 0.0f;
float coSpeed = 0.0f;
float oldCospeed = 0.0f; 
float tp4AngleErrorW = 0.0f;
float oldGyroZAngle = 0.0f;
boolean tp4IsHome = false;
//long tp4HomeDistance = 0L;
float tp4HomeSpeed = 0.0f;
long tp4AccumDistanceError = 0l;
float tp4ControllerSpeed = 0;
int homeTime = 0;
float loopSec = 0.0f;
unsigned int tp4LoopCounter = 0;


// Set up IMU
//  compass.init(LSM303DLHC_DEVICE, 0);
//  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
//  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
//  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
//  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth

/************************************************************************
 *  aTp4Run() 
 *    
 * 
 * 
 ************************************************************************/
void aTp4Run() {
  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  tp4RawIError = 0.0f;
  motorInitTp();
  while(mode == MODE_TP4) { // main loop
    readXBee();  // Read commands from PC or Hand Controller
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    flushSerial();
    checkMotorRight();
    checkMotorLeft();
    dump();

    // Do the timed loop
    if(timeMicroseconds > timeTrigger) {  // Loop executed every XX microseconds 
      actualLoopTime = timeMicroseconds - oldTimeTrigger;
      loopSec = ((float) actualLoopTime)/1000000.0; 
      timeTrigger += 10000; // 100 per second
      oldTimeTrigger = timeMicroseconds;
      
      aTp4(); 
      battery();
      led();
      safeAngle();
      gravity();
      controllerConnected();
      setTp4RunningState();
      cmdBits();
      //    checkDrift();
    } // end timed loop
  }
}



/************************************************************************
 *  aTp4() 
 *    
 * 
 * 
 ************************************************************************/
void aTp4() {
  readSpeed();
  getTpAngle();

  // compute the coSpeed
  float rateCos = wheelSpeedFps + ((*currentValSet).v * gyroXAngleDelta); // subtract out rotation **************
  coSpeed = ((rateCos * (*currentValSet).w)) + ((1.0f - (*currentValSet).w) * oldCospeed); // smooth it out a little
  oldCospeed = coSpeed;

  tp4ControllerSpeed = controllerY * 3.0; //+-3.0 fps

  // find the speed error
  float tp4SpeedError = tp4ControllerSpeed - coSpeed;

  // compute a weighted angle to eventually correct the speed error
  float tp4TargetAngle = tp4SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  float tp4AngleError = gaXAngle - tp4TargetAngle;
  tp4AngleErrorW = tp4AngleError * (*currentValSet).y; //******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target speed.
  tp4Fps = tp4AngleErrorW + coSpeed;
  tp4FpsRight = tp4FpsLeft = tp4Fps;

  tp4Steer();
  setTargetSpeedRight(tp4FpsRight);
  setTargetSpeedLeft(tp4FpsLeft);

  // Send
  if (((++tp4LoopCounter % 5) == 0) || ((tpState & TP_STATE_STREAMING) != 0)) {
    sendArray[TP_SEND_STATE_STATUS] = tpState;
    sendArray[TP_SEND_MODE_STATUS] = mode;
    set2Byte(sendArray, TP_SEND_BATTERY, batteryVolt);
    set4Byte(sendArray, TP_SEND_DEBUG, debugVal);
    if (isBitSet(cmdState, CMD_STATE_STREAM)) {
      set4Byte(sendArray, TP_SEND_A_VAL, timeMicroseconds);
      set4Byte(sendArray, TP_SEND_B_VAL, tickDistanceRight + tickDistanceLeft);
      set2Byte(sendArray, TP_SEND_C_VAL, gyroXRate);
      set2Byte(sendArray, TP_SEND_D_VAL, ay);
      set2Byte(sendArray, TP_SEND_E_VAL, az);
      sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_F_VAL); 
    } 
    else {
      sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_A_VAL); 
    }
  } 
} // end aTp4() 



void tp4Steer() {
  float speedAdjustment = 0.0f;
  boolean controllerZero = abs(controllerX) < 0.05;

  // Adjust straightMode status
  if (straightMode) {
    if (!controllerZero) {
      straightMode = false;  // take it out of straightmode
    } 
    else if (isRotating) {
      speedAdjustment = getRotation();
    }
    else if ((tickDistanceRight - tickDistanceLeft) < tpPositionDiff) {
      speedAdjustment = -.1;
    } 
    else {
      speedAdjustment = +.1;
    }
  }
  else { // not in straightMode
    if (controllerZero) {  // Put in StraightMode if controller is zero
      straightMode = true;  // put it in straightmode
      tpPositionDiff = tickDistanceRight - tickDistanceLeft;
      isRotating = false;
      rotateTarget = 0.0f;
    }
  }

  if (!straightMode) {
    //    speedAdjustment = controllerX * 1.0 - (coSpeed * 0.1); // factor for turning rate.
    speedAdjustment = controllerX * 0.6; // factor for turning rate.
  }

  tp4FpsLeft += speedAdjustment;
  tp4FpsRight -= speedAdjustment;
}



float getRotation() {
  float rotateError = rotateTarget - gyroZAngle;
  if (abs(rotateError) < 1.0f) {
    isRotating = false;
    tpPositionDiff = tickDistanceRight - tickDistanceLeft;
  }
  float rotationRate = rotateError * 0.1f;
  rotationRate = constrain(rotationRate, -2.0f, 2.0f);
  return rotationRate;
}




