//
///************************************************************************
// *  steerRoute() Uses wheel ticks.  Called every timed loop (10 millisec.)
// ************************************************************************/
//void steerRoute() {
//  float speedAdjustment = 0.0;
//  float magDiff = 0.0;
//  float aDiff = 0.0;
//
//  switch (routeCurrentAction) {
//  case 'M': // Mag
//    fixHeading(magHeading, targetHeading);
//    tickDistanceRight = 0;
//    tickDistanceLeft = (long) (magHeading * TICKS_PER_YAW_DEGREE); 
//    break;
//  case 'R':
//    routeReset(false);
//    fixHeading(tickHeading, targetHeading);
//    // Fall through.
//  case 'S': // Speed  // TODO tune this?
//    aDiff = targetHeading - tickHeading;
//    fixHeading(tickHeading, targetHeading);
//    break;
//  case 'T':  // Turn
//    tp5FpsLeft = tp5Fps + turnSpeedAdjustment;
//    tp5FpsRight = tp5Fps - turnSpeedAdjustment;  
//    break;
//  case 'Z':  // Zero speed at current bearing.
//    fixHeading(tickHeading, targetHeading); 
//    break; 
//  }  
//}
//
//
///************************************************************************
// *  routeReset() do it here rather than in switch just to break it out.
// *               Averages Mag readings for 6 seconds before start button 
// *               is pressed until 1 second before button is pressed.
// *               We do calculations in timed loop to get consistent average.
// ************************************************************************/
//void routeReset(boolean r) { 
//  static const int MCOUNT_MAX = 500;
//  static float sum;
//  static int mCount = 0;
//  static int wait = 0;
//
//  if (r == true) { // Start from beginning?
//    mCount = 0;
//    sum = 0;
//    wait = 0;
//  }
//  if ((wait++ > 100) && (mCount < MCOUNT_MAX)) {  
//    sum += magHeading;
//    mCount++;
//    if (mCount >= MCOUNT_MAX) {
//      float deg = sum / ((float) MCOUNT_MAX);
//      tickDistanceRight = 0;
//      tickDistanceLeft = (long) (deg * TICKS_PER_YAW_DEGREE); 
//    }
//  }
//  magCorrection = 0.0;
//}
//
//
//float tp5GetRotation() {
//  float rotateError = rotateTarget - gyroYawAngle;
//  if (abs(rotateError) < 1.0f) {
//    isRotating = false;
//    tpDistanceDiff = tickDistanceRight - tickDistanceLeft;
//  }
//  float rotationRate = rotateError * 0.1f;
//  rotationRate = constrain(rotationRate, -2.0f, 2.0f);
//  return rotationRate;
//}
//
///************************************************************************
// *  Route() Check if target is reached and need to move to next action.
// ************************************************************************/
//void route() {
//  float tbd;
//  if (!isRouteInProgress) return;
//  boolean isNewAction = false;
//  switch (routeCurrentAction) {
//  case 'R':
//    if (digitalRead(YE_SW_PIN) == LOW) isNewAction = true;
//    if (digitalRead(RIGHT_HL_PIN) == HIGH) isNewAction = true;
//    if (routeResetTime < timeMilliseconds) isNewAction = true;
//    break;
//  case 'M':
//    if (abs(magHeading - targetHeading) < 1.0) isNewAction = true;
//    break;
//  case 'S':
//    if (controllerY > 0.0) {
//      if (tickDistance > targetTickDistance) isNewAction = true;
//    }
//    else {
//      if (tickDistance < targetTickDistance) isNewAction = true;
//    }
//    break;
//  case 'T':
//    tbd =  (targetHeading - tickHeading);
//    if (tbd < 180) tbd += 360.0;
//    if (tbd > 180) tbd -= 360.0;
//    if (clockwise) {
//      if (tbd > 0) isNewAction = true;
//    }
//    else {
//      if (tbd < 0) isNewAction = true;
//    }
//    break;
//  case 'Z':
//    if (timeMilliseconds > routeTargetTime) isNewAction = true;
//    break;
//  } // end switch()
//
//  // Move to new action if current action is done.
//  if (isNewAction) { // Move to next action in list.  
//    setNewRouteAction();
//  }
//}
//
//
//
///************************************************************************
// *  setNewRouteAction() Start the next action in the route
// ************************************************************************/
//void setNewRouteAction() {
//  static float k = 2; // Adjust k to get one wheel stopped with radius of 0.5
//  float s, sa, radius, tbd;
//  float motA, motB;
//
//  if (routeActionPtr >= routeActionSize) {
//    isRouteInProgress = false;
//    return;
//  }
//  routeCurrentAction = actionArray[routeActionPtr];
//  int aVal = aValArray[routeActionPtr];
//  int bVal = bValArray[routeActionPtr];
//
//  switch (routeCurrentAction) {
//  case 'M': // Mag Heading
//    targetHeading = ((float) aVal) / 10.0;
//    break;
//  case 'R':   // Reset
//    controllerY = 0.0;
//    routeResetTime = (aVal * 100); // assume seconds are * 10
//    if (routeResetTime == 0) routeResetTime = 1000000L; // zero means wait
//    if (routeResetTime < 7000) routeResetTime = 7000; // need at least 7 sec.
//    routeResetTime += timeMilliseconds;
//    routeReset(true); // get it started.
//    break;
//  case 'S': // Speed
//    controllerY = (((float) aVal) / 100.0) / SPEED_MULTIPLIER;
//    targetTickDistance = tickDistance + (bVal * 10);
//    break;
//  case 'T': // Turn
//    // Degrees go from 0 to 369.9 in .1 increments (unsigned int).
//    targetHeading = ((float) aVal) / 10.0;
//    tbd =  (targetHeading - tickHeading);
//    if (tbd < 180) tbd += 360.0;
//    if (tbd > 180) tbd -= 360.0;
//    if (tbd < 0.0) clockwise = true;
//    else clockwise = false;
//    // radius calculations
//    s = controllerY * SPEED_MULTIPLIER;
//    radius = ((float) bVal) * 0.2;
//    turnSpeedAdjustment = s/radius;
//    if (clockwise) turnSpeedAdjustment = turnSpeedAdjustment * -1.0;
//    break;
//  case 'Z':
//    controllerY = 0.0;
//    routeTargetTime = (aValArray[routeActionPtr] * 10) + timeMilliseconds;
//    break;
//  }
//
//  if (routeActionPtr == 0) { // First call in a route.
//    tickDistanceRight = tickDistanceLeft = tickDistance = 0L;
//  }
//  routeActionPtr++;
//}
//
//
//

