/************************************************************************
 *  Route() called every loop
 *          1. Check if target reached.
 *          2. Set currentLoc.
 *          3. Adjust steering.
 ************************************************************************/

// square in office
//struct routeStep routeA[] = {
//  {'M', -45,   0,  0,   0}, 
//  {'S',   0,   4,  0,   0}, 
//  {'Y',   0,   5,  2,   4},
//  {'T',  -6,   5,  2,   1},
//  {'X',  -6,   5,  2,  -5},
//  {'T',  -6,  -1,  2,   1},
//  {'Y',  -6,  -1,  2,   0},
//  {'T',   0,  -1,  2,   1},
//  {'X',   0,  -1,  2,  -1},
//  {'T',   0,   5,  2,   1},
//  {'F', 0, 0, 0, 0}
//};
//
//// toward kitchen
//struct routeStep routeA[] = {
//  {'M', -70,   0,  0,   0}, 
//  {'S',   0,   4,  0,   0}, 
//  {'Y',   0,   12,  2,   8},
//  {'T',  200,   8.5,  2,   1},
//  {'X',  200,   8.5,  2,  15},
//  {'S',  200,   8.5,  2,   0},
//  {'S', -200,   8.5,  3,   0},
//  {'X', -200,   8.5,  2,   0},
//  {'T',   0, -200,  2,   1},
//  {'Y',   0, -200,  2,   0},
//  {'F',   0,   0,  0,   0}
//};
//

// back and forth along wall to calibrate
// gyroscope turns
struct routeStep routeA[] = {
//  {'M', -70,      0,    0,   0}, 
  {'S',   100,  -100,  0.5,   0}, 
  {'S',   100,    0,    2,   0}, 
  {'X',   100,    0,    3,  12},
  {'S',  -100,   100,  0.5,   0},
  {'S',  -100,    0,    2,   0},
  {'X',  -100,    0,    3,   0},
  {'R',     1,   0,     0,   0}
};



void route() {
  boolean isNewAction = false;
  setPosition();
  
  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'S': // Stand
      if (routeA[routeActionPtr].c < 0.01) {
        if (digitalRead(GN_SW_PIN) == LOW) isNewAction = true;
        if (digitalRead(YE_SW_PIN) == LOW) isNewAction = true;
        if (isEsReceived) {
          isNewAction = true;
          isEsReceived = false;
        }
      }
      else {
        if (routeWaitTime < timeMilliseconds) isNewAction = true;
      }
      steerHeading();
      break;
    case 'X': // Move tpward X axis
      if (isRouteTargetIncreasing) {
        if (routeCurrentLoc.x >= routeTargetXY) isNewAction = true;
      }
      else {
        if (routeCurrentLoc.x <= routeTargetXY) isNewAction = true;
      }
      steerHeading();
      break;
    case 'Y': // Move toward Y axis
      if (isRouteTargetIncreasing) {
        if (routeCurrentLoc.y >= routeTargetXY) isNewAction = true;
      }
      else {
        if (routeCurrentLoc.y <= routeTargetXY) isNewAction = true;
      }
      steerHeading();
      break;
    case 'T': // Turn
      if (abs(routeTargetBearing - routeHeading) < 5.0)  isNewAction = true;
      turn();
      break;
    default:
      break; 
  } // end switch()

  // Move to new action if current action is done.
  if (isNewAction) { // Move to next action in list.  
    setNewRouteAction(false);
  }
}



/************************************************************************
 *  setNewRouteAction() Start the next action in the route
 ************************************************************************/
void setNewRouteAction(boolean start) {

  if (start) {
    routeActionPtr = 0;
    zeroHeadings();
    mapOrientation = 0.0;
    routeCurrentLoc.x = 0;
    routeCurrentLoc.y = 0;
    routeOldTickPosition = 0;
    
  }
  else {
    routeActionPtr++;
  }

  routeCurrentAction = routeA[routeActionPtr].cmd;
  float aRouteVal = routeA[routeActionPtr].a;
  float bRouteVal = routeA[routeActionPtr].b;
  float cRouteVal = routeA[routeActionPtr].c;
  float dRouteVal = routeA[routeActionPtr].d;
  switch (routeCurrentAction) {
    case 'M': // Map Orientation
      mapOrientation = aRouteVal;
      setNewRouteAction(false); // Recursion?
      break;
    case 'F': // Fini
      isRouteInProgress = false;
      break;
    case 'S': // Stand
      routeTargetLoc.x = (float) aRouteVal;
      routeTargetLoc.y = (float) bRouteVal;
      if (cRouteVal < 0.01) { // Zero: Wait for button press
        routeWaitTime = UNSIGNED_LONG_MAX;
      }
      else { // > 0.0: get the time.
        routeWaitTime = ((unsigned int) (cRouteVal * 1000.0)) + timeMilliseconds;
      }
      routeFps = 0;
      break;
    case 'X':   // Move
    case 'Y':   // Move
      routeTargetLoc.x = (float) aRouteVal;
      routeTargetLoc.y = (float) bRouteVal;
      if (routeCurrentAction == 'X') {
        if (routeCurrentLoc.x < routeTargetLoc.x) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      else {
        if (routeCurrentLoc.y < routeTargetLoc.y) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      routeFps = cRouteVal;
      routeTargetXY = dRouteVal;
      break;
    case 'T':   // Turn
      routeTargetLoc.x = aRouteVal;
      routeTargetLoc.y = bRouteVal;
      routeRadius = dRouteVal;
      routeFps = cRouteVal;
      break;
    case 'R': // Repeat.
      routeActionPtr = (int) aRouteVal;
      setNewRouteAction(false); // Recursion?
      break;
    default:
      break;
  }
}



/************************************************************************
 *  steerHeading() Find the correct heading to the target and adjust the 
 *               wheel speeds to turn toward the target.  The adjustments
 *               take place at the specified radius.
 ************************************************************************/
#define A_LIM 10.0 // degrees at which the speedAdjustment starts reducing.
#define S_LIM 1.0  // maximum speedAdjustment;
void steerHeading() {
  aDiff = routeTargetBearing - routeHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  
  speedAdjustment = aDiff * (S_LIM / A_LIM);
  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
    
  tp6FpsLeft = tp6Fps + speedAdjustment;
  tp6FpsRight = tp6Fps - speedAdjustment;
}



/************************************************************************
 *  turn()
 ************************************************************************/
void turn() {
  aDiff = routeTargetBearing - routeHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  float d = (aDiff > 0.0) ? 1.0 : -1.0;
  
  speedAdjustment = (routeFps / routeRadius) * 0.6 * d; 
  
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  setPosition() Set the new XY position given the distance and
 *                orientation traveled since the last call.
 ************************************************************************/
void setPosition() {
  // Set the current heading
  float h = gyroHeading - mapOrientation;
  if (h > 180.0) h -= 360.0;
  else if (h < -180.0) h += 360.0;
  routeHeading = h;
  
  // Set the current location
  double dist = ((double) (tickPosition - routeOldTickPosition)) / TICKS_PER_FOOT;
  routeOldTickPosition = tickPosition;
  routeCurrentLoc.x += sin(routeHeading * DEG_TO_RAD) * dist;
  routeCurrentLoc.y += cos(routeHeading * DEG_TO_RAD) * dist;
  
  // Set the bearing to the target
  float x =  routeTargetLoc.x - routeCurrentLoc.x;
  float y = routeTargetLoc.y - routeCurrentLoc.y;
  routeTargetBearing = atan2(x,y) * RAD_TO_DEG;
}

