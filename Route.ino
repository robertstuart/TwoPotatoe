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
////// toward kitchen
//struct routeStep routeA[] = {
//  {'M',  -51,   0,  0,   0},  // 0
//  {'S',    0,   6, 30,   0}, // 1 
//  {'Y',    0,   4,  2,   0},  // 2
//  {'T',   15,   6,  2,   2},  // 3
//  {'X',   15,   6,  2,   0},  // 4
//  {'x',  5.2,   4,  7, 0.6},
//  {'x', 11.8,   2,  5, 3.3},
//  {'S',   20,   6,  1,   0},  // 5
//  {'S',    2,   6,  2,   0},  // 6
//  {'X',    2,   6,  2,   0},  // 7
//  {'x', 13.5,   2,  4, 2.3},
//  {'x',  4.8,   2,  4, 2.3},
//  {'T',    0,   0,  2,   2},  // 8
//  {'Y',    0,   0,  2,   0},  // 9
//  {'F',    0,   0,  0,   0}
//};
//


// turns
struct routeStep routeA[] = {
  {'M',  -51,   0,  0,   0},  // 0
  {'S',    0,   10,  2,   0}, // 1 
  
  {'T',    10,   0,  2,   1},  // 8 CW
  {'T',     0, -10,  4,   1},  // 8
  {'T',   -10,   0,  4,   1},  // 8
  {'T',    0,   10,  4,   1},  // 3
  
  {'T',    10,   0,  4,   1},  // 8
  {'T',     0, -10,  4,   1},  // 8
  {'T',   -10,   0,  4,   1},  // 8
  {'T',    0,   10,  2,   1},  // 3
  
//  {'T',    -10,   0,  2,   1},  // 8 CCW
//  {'T',     0, -10,  4,   1},  // 8
//  {'T',    10,   0,  4,   1},  // 8
//  {'T',    0,   10,  4,   1},  // 3
//  
//  {'T',    -10,   0,  4,   1},  // 8
//  {'T',     0, -10,  4,   1},  // 8
//  {'T',    10,   0,  4,   1},  // 8
//  {'T',    0,   10,  2,   1},  // 3
  
  {'F',    0,   0,  0,   0}
};



//// back and forth to calibrate
//// gyroscope turns CCW
//struct routeStep routeA[] = {
//  {'M',   -70,      0,    0,   0}, 
//  {'S',   15,   -10,  0.5,   0}, 
//  {'S',   15,     0,    2,   0}, 
//  {'X',   15,     0,    3,   0},
//  {'S',    0,    10,  0.5,   0},
//  {'S',    0,     0,    2,   0},
//  {'X',    0,     0,    3,   0},
//  {'R',    1,     0,    0,   0}
//};


// back and forth to calibrate
// gyroscope turns CW
//struct routeStep routeA[] = {
//  {'M',   -70,      0,    0,   0}, 
//  {'S',   15,    10,  0.5,   0}, 
//  {'S',   15,     0,    2,   0}, 
//  {'X',   15,     0,    3,   0},
//  {'S',    0,   -10,  0.5,   0},
//  {'S',    0,     0,    2,   0},
//  {'X',    0,     0,    3,   0},
//  {'R',    1,     0,    0,   0}
//};



void route() {
  boolean isNewAction = false;
  setTargetPosition();
  
  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'S': // Stand
      if (digitalRead(GN_SW_PIN) == LOW) isNewAction = true;
      if (digitalRead(YE_SW_PIN) == LOW) isNewAction = true;
      if (isEsReceived) {
        isNewAction = true;
        isEsReceived = false;
      }
      if (routeA[routeActionPtr].c > 0.01) {
        if (routeWaitTime < timeMilliseconds) isNewAction = true;
      }
      steerHeading();
      break;
    case 'x': // Moving toward X axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x; 
      routeCoDistance = routeCoDistanceXY - currentMapLoc.x;
      if (isRouteTargetIncreasing) {
        if (routeCoDistance <= 0.0) {
          isNewAction = true;
          readSonar();
        }
      }
      else {
        if (routeCoDistance >= 0.0) {
          isNewAction = true;
          readSonar();
        }
      }
      steerHeading();
      break;
    case 'X': // Move toward X axis
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x; 
      if (isRouteTargetIncreasing) {
        if (routeTargetXYDistance <= 0.0) isNewAction = true;
      }
      else {
        if (routeTargetXYDistance >= 0.0) isNewAction = true;
      }
      steerHeading();
      break;
    case 'y': // Moving toward Y axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y; 
      routeCoDistance = routeCoDistanceXY - currentMapLoc.y;
      if (isRouteTargetIncreasing) {
        if (routeCoDistance <= 0.0) { 
          isNewAction = true;
          readSonar();
        }
      }
      else {
        if (routeCoDistance >= 0.0) {
          isNewAction = true;
          readSonar();
        }
      }
      steerHeading();
      break;
    case 'Y': // Move toward Y axis
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y; 
      if (isRouteTargetIncreasing) {
        if (routeTargetXYDistance <= 0.0) isNewAction = true;
      }
      else {
        if (routeTargetXYDistance >= 0.0) isNewAction = true;
      }
      steerHeading();
      break;
    case 'T': // Turn
      if (abs(routeTargetBearing - currentMapHeading) < 5.0)  isNewAction = true;
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
    resetRoute();
  }
  else {
    // Revert to X or Y if current is x or y and next is something else.
    if ((routeCurrentAction == 'x') || (routeCurrentAction == 'y')) {
      int nextAction = routeA[routeActionPtr + 1].cmd;
      if ((nextAction != 'x') && (nextAction != 'y')) {
        // We are here if we have reached the last in a series of "x or y" steps
        // Revert to acting as if the next command is the original X or Y command.
        routeCurrentAction = originalAction;
        return;
      }
    }
    routeActionPtr++;
  }
  
  routeCurrentAction = routeA[routeActionPtr].cmd;
  double aRouteVal = routeA[routeActionPtr].a;
  double bRouteVal = routeA[routeActionPtr].b;
  double cRouteVal = routeA[routeActionPtr].c;
  double dRouteVal = routeA[routeActionPtr].d;
  switch (routeCurrentAction) {
    case 'M': // Map Orientation
      mapOrientation = aRouteVal;
      setNewRouteAction(false); // Recursion?
      break;
    case 'F': // Fini
      isRouteInProgress = false;
      break;
    case 'S': // Stand
      routeTargetLoc.x = (double) aRouteVal;
      routeTargetLoc.y = (double) bRouteVal;
      if (cRouteVal < 0.01) { // Zero: Wait for button press
        routeWaitTime = UNSIGNED_LONG_MAX;
      }
      else { // > 0.0: get the time.
        routeWaitTime = ((unsigned int) (cRouteVal * 1000.0)) + timeMilliseconds;
      }
      routeFps = 0;
      routeTargetXYDistance = 9999.9; // always turn.
      isEsReceived = false;
      break;
    case 'X':   // Move
      routeTargetXY = routeTargetLoc.x = (double) aRouteVal;
      routeTargetLoc.y = (double) bRouteVal;
      if (currentMapLoc.x < routeTargetLoc.x) isRouteTargetIncreasing = true;
      else isRouteTargetIncreasing = false;
      routeFps = cRouteVal;
      if (routeA[routeActionPtr + 1].cmd == 'x') {
        originalAction = 'X';
        setNewRouteAction(false); // Recursion?
      }
      break;
    case 'x':
    case 'y':
      routeCoDistanceXY = aRouteVal;
      routeSonarMin = bRouteVal;
      routeSonarMax = cRouteVal;
      routeSonarDist = dRouteVal;
      break;
    case 'Y':   // Move
      routeTargetLoc.x = (double) aRouteVal;
      routeTargetXY = routeTargetLoc.y = (double) bRouteVal;
      if (currentMapLoc.y < routeTargetLoc.y) isRouteTargetIncreasing = true;
      else isRouteTargetIncreasing = false;
      routeFps = cRouteVal;
      if (routeA[routeActionPtr + 1].cmd == 'y') {
        originalAction = 'Y';
        setNewRouteAction(false); // Recursion?
      }
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
 *  readSonar() 
 *           
 *           
 ************************************************************************/
 void readSonar() {
  double diff = 0.0;
  sonarRight = ((double) analogRead(SONAR_RIGHT_AN)) * SONAR_SENS; // to feet
  if ((sonarRight > routeSonarMin) && (sonarRight < routeSonarMax)) {
    diff = routeSonarDist - sonarRight;
//    if (routeCurrentAction == 'x') currentMapLoc.y += diff;
//    else currentMapLoc.x += diff;
  }
  BLUE_SER.print(sonarRight);
  BLUE_SER.print("\t");
  BLUE_SER.print(routeSonarDist);
  BLUE_SER.print("\t");
  BLUE_SER.print(currentMapLoc.x);
  BLUE_SER.print("\t");
  BLUE_SER.print(currentMapLoc.y);
  BLUE_SER.print("\t");
  BLUE_SER.println();
  
}


/************************************************************************
 *  steerHeading() Find the correct heading to the target and adjust the 
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 ************************************************************************/
#define A_LIM 10.0 // degrees at which the speedAdjustment starts reducing.
#define S_LIM 1.0  // maximum speedAdjustment;
void steerHeading() {
  static double oldTb = 0.0;
  double tb;
  if (abs(routeTargetXYDistance) < 1.0) { // No turning when closer than X.X feet.
    tb = oldTb;
  }
  else {
    oldTb = tb = routeTargetBearing;
  }
  aDiff = tb - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  
  speedAdjustment = aDiff * (S_LIM / A_LIM);
  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
    
  tp6FpsLeft = tp6Fps + speedAdjustment;
  tp6FpsRight = tp6Fps - speedAdjustment;
}



/************************************************************************
 *  turn()
 *  Why does this turn at a different depending on direction???????????????????????????????
 ************************************************************************/
void turn() {
  aDiff = routeTargetBearing - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  double d = (aDiff > 0.0) ? 1.0 : -1.0;
  
  speedAdjustment = (routeFps / routeRadius) * 0.37 * d; 
  
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  setPosition() Set the new XY position given the distance and
 *                orientation traveled since the last call.
 ************************************************************************/
void setTargetPosition() {
  // Set the bearing to the target
  double x =  routeTargetLoc.x - currentMapLoc.x;
  double y = routeTargetLoc.y - currentMapLoc.y;
  routeTargetBearing = atan2(x,y) * RAD_TO_DEG;
}

void resetRoute() {
  resetNavigation();
  routeActionPtr = 0;
}
