/************************************************************************
 * ----- TP4 -----
 *    Algorithm using the TP motor controller
 * 
 * 
 ************************************************************************/
#define PID1_LOOP_TIME 10000L;        // Microsecons/loop


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

void aTp4Init() {
  tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
  tp4RawIError = 0.0f;
  loopTime = PID1_LOOP_TIME;
  motorInitTp();
}

void aTp4Start() {
  tp4RawIError = 0.0;  // Get rid of any accumulated error.
  tp4IsHome = false;
}



void aTp4() {
  tp4Home();
  // compute the coSpeed
  float rateCos = wheelSpeedFps + ((*currentValSet).v * gyroXAngleDelta); // subtract out rotation **************
  coSpeed = ((rateCos * (*currentValSet).w)) + ((1.0f - (*currentValSet).w) * oldCospeed); // smooth it out a little
  oldCospeed = coSpeed;

  // get the target speed
  if (home == 0L) {
    tp4ControllerSpeed = controllerY * 3.0; //+-3.0 fps
  }
  else {
    tp4ControllerSpeed = tp4HomeSpeed;
  }

  // find the speed error
  float tp4SpeedError = tp4ControllerSpeed - coSpeed;

  // compute a weighted angle to eventually correct the speed error
  float tp4TargetAngle = tp4SpeedError * (*currentValSet).x; //************ Speed error to angle *******************

  // limit the angle
//  tp4TargetAngle = constrain(tp4TargetAngle, -15.0f, 15.0f);

  // Compute angle error and weight factor
  float tp4AngleError = gaXAngle - tp4TargetAngle;
  tp4AngleErrorW = tp4AngleError * (*currentValSet).y; //******************* Angle error to speed *******************

  // Add the angle error to the base speed to get the target speed.
  tp4Fps = tp4AngleErrorW + coSpeed;
  tp4FpsRight = tp4FpsLeft = tp4Fps;


  tp4Steer();
  setTargetSpeedRight(tp4FpsRight);
  setTargetSpeedLeft(tp4FpsLeft);
  //  setTargetSpeedRight(pid2FpsTwsRight);
  //  setTargetSpeedLeft(pid2FpsTwsLeft);

  // Set the debug values
  //aVal = timeMicroSeconds/1000000.0f;
  aVal = tickDistance;
  bVal = gaXAngle;
  cVal = wheelSpeedFps;
  dVal = gyroXAngleDelta;
  eVal = coSpeed;
  fVal = tp4ControllerSpeed;
  gVal = tp4TargetAngle;
  hVal = tp4AngleErrorW;
  iVal = tp4ISpeed;
  jVal = tp4Fps;
//  jVal = gyroZAngle;
  //  debugFloat("tp4Fps: ", tp4Fps);
} // End tp4();

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



void tp4Home() {

  tp4HomeSpeed = 0.0f;
  if (home == 0L) {
    return;
  }
  long distanceError = home - (tickDistanceRight + tickDistanceLeft);
  tp4HomeSpeed = (distanceError * .0005);
  
  
//  if (tp4IsHome) {
//    // first check to see if we need to exit this mode
//    if ((abs(controllerY) > 0.05) || (abs(gaXAngle) > 15.0)) {
//      tp4IsHome = false;
//      homeTime = 0;
//      debugBoolean("tp4IsHome: ", tp4IsHome);
//    }
//    else {
//      long distanceError = tp4HomeDistance - (tickDistanceRight + tickDistanceLeft);
//      tp4HomeSpeed = (distanceError * .0005);
//    }
//  }
//  else if ((abs(controllerY) <= 0.05) && (abs(gaXAngle) <= 15.0)) {
//    if (homeTime > 150) { // Wait at least a second.
//      tp4IsHome = true;
//      tp4HomeDistance = tickDistanceRight + tickDistanceLeft;
//      debugBoolean("tp4IsHome: ", tp4IsHome);
//    } 
//    else {
//      homeTime++;
//    }
//  } 
//  else {
//    homeTime = 0;
//  }
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


