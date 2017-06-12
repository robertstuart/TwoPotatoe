const float SEEK_FPS = 2.0;
const float TURN_FPS = 3.0;
// Barrel strategy
const int BARREL_STRATEGY1 = 0;
const int BARREL_STRATEGY2 = 1;
int barrelStrategyState = BARREL_STRATEGY1;

// Barrel states
const int SEEK_BARREL = 0;
const int PLOT_BARREL = 1;
const int CIRCLE_BARREL = 2;
int barrelOneState = SEEK_BARREL;

// Pivot states
const int PIVOT_SETTLE = 0;
const int PIVOT_LEFT = 1;
const int PIVOT_RIGHT = 2;
int plotState = PIVOT_SETTLE;

// Circle states
const int CIRCLE_ORIENT = 0;
const int CIRCLE_TURN = 2;
int circleState = CIRCLE_ORIENT;

struct sonarPoint {
  float head;
  float distance;
};

const int SONAR_SIZE = 200;
struct sonarPoint sonarPoints[SONAR_SIZE];
int sonarPtr = 0;
int sonarEnd = 0;
struct sonarPoint sonarA = { 0.0, 0.0 };
struct sonarPoint sonarB = { 0.0, 0.0 };

struct loc barrelLoc;
float barrelHead = 0.0;
float barrelDist = 0.0;
struct loc seekLoc;
boolean isWestOfCenter = true;
float barrelEdgeAngle = 0.0;

/************************************************************************.
    barrels() 
 ************************************************************************/
boolean barrels(boolean reset) {
  if (reset) {
    barrelStrategyState = BARREL_STRATEGY1;
    barrelOne(true);  // reset it
    barrelTwo(true);  // reset it
  } else {
    if (barrelStrategyState == BARREL_STRATEGY1) {
      return barrelOne(false);
    } else {
      return barrelTwo(false);
    }
  }
  return true; // just in case.
}



/************************************************************************.
    barrelOne() Strategy #1
                Returns true if endY reached.
 ************************************************************************/
boolean barrelOne(boolean reset) {
  float speedAdjustment;
//  addLog(
//    (long) (timeMilliseconds),
//    (short) (currentLoc.x * 100.0),
//    (short) (currentLoc.y * 100.0),
//    (short) (barrelOneState),
//    (short) (gyroHeading * 100.0),
//    (short) (barrelYEnd * 100.0),
//    (short) (sonarFront * 100.0)
//  );
  
  if (reset) {
    barrelX = currentLoc.x;
    barrelOneState = SEEK_BARREL;
    routeFps = 1.0;
    barrelX = currentLoc.x;   // Use arrival X
    return false;
  }
  
  switch (barrelOneState) {
    case SEEK_BARREL:
      if (currentLoc.y > barrelYEnd) return true;
      if (abs(gyroHeading) > 5.0) {
        routeFps = 0.0;
        speedAdjustment = (gyroHeading > 0.0) ? 0.3 : -0.3;
        targetFpsRight = targetFps + speedAdjustment;
        targetFpsLeft = targetFps - speedAdjustment;
     } else {
        if (sonarFront < 2.5) {
          barrelOneState = PLOT_BARREL;
          plotBarrel(true);  //reset
        } else {
          routeFps = SEEK_FPS;
          targetLoc.x = barrelX;
          targetLoc.y = currentLoc.y + 2.0;
          setTarget();
          steerHeading();
        }
      }
      break;

    case PLOT_BARREL:
      plotBarrel(false);
      break;

    case CIRCLE_BARREL:
      circleBarrel(false);
      break;

    default:
      break;
  }


  return (false);
}

const float PIVOT_RATE = 0.3;
const float HEADING_LIMIT = 45.0;
const float SONAR_EDGE = 4.0;
/************************************************************************.
 *  plotBarrel()
 ************************************************************************/
void plotBarrel(boolean reset) {
  static boolean isSonarA = false;
  static int startTime = 0;
  static int pivotCount = 0;

  if (reset) {
    isWestOfCenter = (currentLoc.x < barrelXCenter) ? true : false;
    plotState = PIVOT_SETTLE;
    seekLoc = currentLoc;
    startTime = timeMilliseconds;
    sonarEnd = 0;
    sonarA.distance = 0.0;
    sonarB.distance = 0.0;
    pivotCount = 0;
    return;
  }

  holdY();

  switch (plotState) {
    case PIVOT_SETTLE: 
      if ((timeMilliseconds - startTime) > 1000.0) {
        plotState = (isWestOfCenter) ? PIVOT_RIGHT : PIVOT_LEFT;
      }
      break;
    case PIVOT_RIGHT:
      targetFpsRight = targetFps - PIVOT_RATE;
      targetFpsLeft = targetFps + PIVOT_RATE;
      if (gyroHeading > 0.0) {
        if (sonarFront > SONAR_EDGE) {
          barrelEdgeAngle = gyroHeading;
          barrelOneState = CIRCLE_BARREL;
          circleBarrel(true); // reset
        }
      }  
      if (gyroHeading > HEADING_LIMIT) {
        if (pivotCount > 5) {
          barrelStrategyState = BARREL_STRATEGY2;
        }
        pivotCount++;
        plotState = PIVOT_LEFT;
      }
      break;
    case PIVOT_LEFT:
      targetFpsRight = targetFps + PIVOT_RATE;
      targetFpsLeft = targetFps - PIVOT_RATE;
      if (gyroHeading < 0.0) {
        if (sonarFront > SONAR_EDGE) {
          barrelEdgeAngle = gyroHeading;
          barrelOneState = CIRCLE_BARREL;
          circleBarrel(true); // reset
        }
      } 
      if (gyroHeading < -HEADING_LIMIT) {
        if (pivotCount > 5) {
          barrelStrategyState = BARREL_STRATEGY2;
        }
        pivotCount++;
        plotState = PIVOT_RIGHT;
      }
      break;
  }
}



const float CIRCLE_ORIENT_RATE = 0.2;
const float END_TAN_DEGREES = 50.0;
const float TURN_RADIUS = 2.4;
const float PIVOT_TARGET_X = 1.55;
/************************************************************************.
    circleBarrel()
 ************************************************************************/
void circleBarrel(boolean reset) {
  const float DISPLACEMENT_Y = 3.5;
  float displacement;

  if (reset) { // set isRightTurn, turnRadius, pivotLoc, targetLoc
    turnRadius = TURN_RADIUS;
    displacement = (abs(barrelEdgeAngle) * 0.06) - 1.75;
    targetLoc.y = currentLoc.y + 5.0;
    pivotLoc.y = currentLoc.y + 3.2;
    if (barrelEdgeAngle > 0.0) { // go right around barrel
      isRightTurn = false;
      pivotLoc.x = currentLoc.x + displacement;
      targetLoc.x = pivotLoc.x + PIVOT_TARGET_X;
    } else {
      isRightTurn = true;
      pivotLoc.x = currentLoc.x - displacement;
      targetLoc.x = pivotLoc.x -+ PIVOT_TARGET_X;
    }
    routeFps = TURN_FPS;
    sprintf(message, "displacement: %5.2f   barrelEdgeAngle: %5.2f   pivotBearing: %5.2f", displacement, barrelEdgeAngle, pivotBearing);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "targetLoc.x: %5.2f   targetLoc.y: %5.2f   turnRadius: %5.2f", targetLoc.x, targetLoc.y, turnRadius);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "currentLoc.x: %5.2f   currentLoc.y: %5.2f", currentLoc.x, currentLoc.y);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "pivotLoc.x: %5.2f   pivotLoc.y: %5.2f", pivotLoc.x, pivotLoc.y);
    sendBMsg(SEND_MESSAGE, message);
  } else {
    setTarget();
    if (isTargetReached()) {
      plotBarrel(true); // reset
      barrelOneState = SEEK_BARREL;
    } else {
      turn();
    }
  }
}




const float PED_SONAR_LIMIT = 8.0;
const float PED_ANGLE = 25.0;
/************************************************************************
    pedestrian()
 ************************************************************************/
boolean pedestrian(boolean reset) {
  static int pedPivotState = PIVOT_LEFT;
  static int sonarPedCount = 0;
  static int pivotCount = 0;
  if (reset) {
    seekLoc = currentLoc;
    sonarPedCount = 0;
    pivotCount = 0;
    pedPivotState = PIVOT_LEFT;
  } else {
    if (sonarFront > 0.1) {
      if (sonarFront < PED_SONAR_LIMIT) {
        sonarPedCount++;
        sonarFront = 0.0;
      }
    }
    holdY();
    if (pedPivotState == PIVOT_LEFT) {
      if ((pivotCount > 10) && (gyroHeading < 0.0)) return true;
      if (gyroHeading < -PED_ANGLE) {
        if ((sonarPedCount == 0) && pivotCount) return true;
        sonarPedCount = 0;
        pedPivotState = PIVOT_RIGHT;
        pivotCount++;
      }
      targetFpsRight = targetFps + PIVOT_RATE;
      targetFpsLeft = targetFps - PIVOT_RATE;
    } else {
      if (gyroHeading > PED_ANGLE) {
        if ((sonarPedCount == 0) && pivotCount) return true;
        sonarPedCount = 0;;
        pedPivotState = PIVOT_LEFT;
        pivotCount++;
      }
      targetFpsRight = targetFps - PIVOT_RATE;
      targetFpsLeft = targetFps + PIVOT_RATE;
    }
  }
  return false;
}



/************************************************************************.
    barrelTwo() Strategy #2
 ************************************************************************/
boolean barrelTwo(boolean reset) {
  return true;
}



const float HOLD_LIMIT = 0.1;
const float HOLD_SPEED = 0.2;
void holdY() {
  float yError = seekLoc.y - currentLoc.y;
  if (yError < (-HOLD_LIMIT * 2.0)) routeFps = -HOLD_SPEED * 3.0;
  else if (yError > (HOLD_LIMIT * 2.0)) routeFps = HOLD_SPEED * 3.0;
  else if (yError < -HOLD_LIMIT) routeFps = -HOLD_SPEED;
  else if (yError > HOLD_LIMIT) routeFps = HOLD_SPEED;
  else routeFps = 0.0;
}



