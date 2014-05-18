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
    dump();

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

  // compute the tp5CoSpeed
  float rateCos = wheelSpeedFps + ((*currentValSet).v * gyroXAngleDelta); // subtract out rotation **************
  tp5CoSpeed = ((rateCos * (*currentValSet).w)) + ((1.0f - (*currentValSet).w) * tp5OldCospeed); // smooth it out a little
  tp5OldCospeed = tp5CoSpeed;

  tp5ControllerSpeed = controllerY * 3.0; //+-3.0 fps

  // find the speed error
  float tp5SpeedError = tp5ControllerSpeed - tp5CoSpeed;

  // compute a weighted angle to eventually correct the speed error
  float tp5TargetAngle = tp5SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // Compute angle error and weight factor
  float tp5AngleError = gaXAngle - tp5TargetAngle;
  tp5AngleErrorW = tp5AngleError * (*currentValSet).y; //******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target speed.
  tp5Fps = tp5AngleErrorW + tp5CoSpeed;
  tp5FpsRight = tp5FpsLeft = tp5Fps;

  tp5Steer();
  setTargetSpeedRight(tp5FpsRight);
  setTargetSpeedLeft(tp5FpsLeft);

  // Send
  if (((++tp5LoopCounter % 5) == 0) || ((tpState & TP_STATE_STREAMING) != 0)) {
    sendArray[TP_SEND_STATE_STATUS] = tpState;
    sendArray[TP_SEND_MODE_STATUS] = mode;
    set2Byte(sendArray, TP_SEND_BATTERY, batteryVolt);
    set2Byte(sendArray, TP_SEND_DEBUG, debugVal);
    if (isBitSet(tpState, TP_STATE_STREAMING)) {
      set4Byte(sendArray, TP_SEND_A_VAL, timeMicroseconds);
      set4Byte(sendArray, TP_SEND_B_VAL, tickDistanceRight + tickDistanceLeft);
      set2Byte(sendArray, TP_SEND_C_VAL, gyroXRate);
      set2Byte(sendArray, TP_SEND_D_VAL, ay);
      set2Byte(sendArray, TP_SEND_E_VAL, az);
      set2Byte(sendArray, TP_SEND_F_VAL, blinkPattern[blinkPtr]);
      sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_G_VAL); 
    } 
    else {
      sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_A_VAL); 
    }
  } 
} // end aTp5() 



void tp5Steer() {
  float speedAdjustment = 0.0f;
  boolean controllerZero = abs(controllerX) < 0.05;

  // Adjust straightMode status
  if (straightMode) {
    if (!controllerZero) {
      straightMode = false;  // take it out of straightmode
    } 
    else if (isRotating) {
      speedAdjustment = tp5GetRotation();
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
    //    speedAdjustment = controllerX * 1.0 - (tp5CoSpeed * 0.1); // factor for turning rate.
    speedAdjustment = controllerX * 0.6; // factor for turning rate.
  }

  tp5FpsLeft += speedAdjustment;
  tp5FpsRight -= speedAdjustment;
}



float tp5GetRotation() {
  float rotateError = rotateTarget - gyroZAngle;
  if (abs(rotateError) < 1.0f) {
    isRotating = false;
    tpPositionDiff = tickDistanceRight - tickDistanceLeft;
  }
  float rotationRate = rotateError * 0.1f;
  rotationRate = constrain(rotationRate, -2.0f, 2.0f);
  return rotationRate;
}




