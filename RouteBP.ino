float barrelHead = 0.0;
float barrelDist = 0.0;
struct loc seekLoc;
float barrelEdgeAngle = 0.0;

/************************************************************************.
    barrels() 
 ************************************************************************/
boolean barrels(boolean reset) {
  if (reset) {
    barrelOne(true);  // reset it
    return true;      // return not used on reset.
  } else {
    return barrelOne(false);
  } 
}



/************************************************************************.
    barrelOne() Strategy #1
                Returns true if endY reached.
 ************************************************************************/
boolean barrelOne(boolean reset) {
  float speedAdjustment;
  
  if (reset) {
    barrelX = currentLoc.x;   // Use arrival X
    barrelOneState = SEEK_BARREL;
    setSonar("lFr");
    if (isRunning) {
      sprintf(message, "b %6.1f  %4.1f", currentLoc.y, routeScriptFps);
    }
    sendXMsg(SEND_MESSAGE, message);
    return false;
  }
  
  switch (barrelOneState) {
    case SEEK_BARREL:
      if (currentLoc.y > barrelYEnd) return true;
      if (abs(gyroHeading) > 5.0) { // Stop & turn center if not pointed N
        speedAdjustment = (gyroHeading > 0.0) ? 0.3 : -0.3;
        targetWFpsRight = targetWFps + speedAdjustment;
        targetWFpsLeft = targetWFps - speedAdjustment;
     } else {
        if (sonarFront < 2.5) {
          barrelOneState = PLOT_BARREL;
          seekLoc = currentLoc;
          plotBarrel(true);  //reset
       } else {
          targetLoc.x = currentLoc.x;
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



const float PIVOT_RATE = 0.5;
const float HEADING_LIMIT = 45.0;
const float SONAR_EDGE = 4.0;
const int PIVOT_LIMIT = 8;
/************************************************************************.
 *  plotBarrel()  A barrel has been detected.  Look left and right.
 ************************************************************************/
void plotBarrel(boolean reset) {
  static int startTime = 0;
  static int pivotCount = 0;

//  addLog(
//    (long) (barrelOneState),
//    (short) (plotState),
//    (short) (gyroHeading * 100.0),
//    (short) (pivotCount * 100.0),
//    (short) (sonarFront * 100.0),
//    (short) (currentLoc.x * 100.0),
//    (short) (currentLoc.y * 100.0)
//  );
  
  if (reset) {
    plotState = PLOT_SETTLE;
    seekLoc = currentLoc;
    startTime = timeMilliseconds;
    pivotCount = 0;
    return;
  }

  switch (plotState) {
    case PLOT_SETTLE: 
      if ((timeMilliseconds - startTime) > 2000.0) {
        plotState = (currentLoc.x < barrelXCenter) ? PLOT_RIGHT : PLOT_LEFT;
      }
      break;
    case PLOT_RIGHT:
      targetWFpsRight = targetWFps - PIVOT_RATE;
      targetWFpsLeft = targetWFps + PIVOT_RATE;
      if (gyroHeading > 0.0) {
        if (sonarFront > SONAR_EDGE) {
          barrelEdgeAngle = gyroHeading;
          barrelOneState = CIRCLE_BARREL;
          circleBarrel(true); // reset
        }
      } 
      if (gyroHeading > HEADING_LIMIT) {
        if (pivotCount > PIVOT_LIMIT) {
          barrelStrategyState = BARREL_STRATEGY2;
        }
        pivotCount++;
        plotState = PLOT_LEFT;
      }
      break;
    case PLOT_LEFT:
      targetWFpsRight = targetWFps + PIVOT_RATE;
      targetWFpsLeft = targetWFps - PIVOT_RATE;
      if (gyroHeading < 0.0) {
        if (sonarFront > SONAR_EDGE) {
          barrelEdgeAngle = gyroHeading;
          barrelOneState = CIRCLE_BARREL;
          circleBarrel(true); // reset
        }
      } 
      if (gyroHeading < -HEADING_LIMIT) {
        if (pivotCount > PIVOT_LIMIT) {
          barrelStrategyState = BARREL_STRATEGY2;
        }
        pivotCount++;
        plotState = PLOT_RIGHT;
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
    sprintf(message, "displacement: %5.2f   barrelEdgeAngle: %5.2f   pivotBearing: %5.2f", displacement, barrelEdgeAngle, pivotBearing);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "targetLoc.x: %5.2f   targetLoc.y: %5.2f   turnRadius: %5.2f", targetLoc.x, targetLoc.y, turnRadius);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "currentLoc.x: %5.2f   currentLoc.y: %5.2f", currentLoc.x, currentLoc.y);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "pivotLoc.x: %5.2f   pivotLoc.y: %5.2f", pivotLoc.x, pivotLoc.y);
    sendBMsg(SEND_MESSAGE, message);
    sprintf(message, "Reset Circle. currentLoc.x: %5.2f   currentLoc.y: %5.2f  millis: %6d", currentLoc.x, currentLoc.y, timeMilliseconds);
    sendBMsg(SEND_MESSAGE, message);
} else {
    setTarget();
    if (isTargetReached()) {
      barrelOne(true); // reset
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
  static int pedPivotState = PLOT_LEFT;
  static int sonarPedCount = 0;
  static int pivotCount = 0;
  if (reset) {
    seekLoc = currentLoc;
    sonarPedCount = 0;
    pivotCount = 0;
    pedPivotState = PLOT_LEFT;
  } else {
    if (sonarFront > 0.1) {
      if (sonarFront < PED_SONAR_LIMIT) {
        sonarPedCount++;
        sonarFront = 0.0;
      }
    }
    holdY();
    if (pedPivotState == PLOT_LEFT) {
      if ((pivotCount > 10) && (gyroHeading < 0.0)) return true;
      if (gyroHeading < -PED_ANGLE) {
        if ((sonarPedCount == 0) && pivotCount) return true;
        sonarPedCount = 0;
        pedPivotState = PLOT_RIGHT;
        pivotCount++;
      }
      targetWFpsRight = targetWFps + PIVOT_RATE;
      targetWFpsLeft = targetWFps - PIVOT_RATE;
    } else {
      if (gyroHeading > PED_ANGLE) {
        if ((sonarPedCount == 0) && pivotCount) return true;
        sonarPedCount = 0;;
        pedPivotState = PLOT_LEFT;
        pivotCount++;
      }
      targetWFpsRight = targetWFps - PIVOT_RATE;
      targetWFpsLeft = targetWFps + PIVOT_RATE;
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
/************************************************************************.
    holdY() Set routeFps to seek to y value of seekLoc.
 ************************************************************************/
void holdY() {
  float yError = seekLoc.y - currentLoc.y;
  if (yError < (-HOLD_LIMIT * 2.0)) routeFps = -HOLD_SPEED * 3.0;
  else if (yError > (HOLD_LIMIT * 2.0)) routeFps = HOLD_SPEED * 3.0;
  else if (yError < -HOLD_LIMIT) routeFps = -HOLD_SPEED;
  else if (yError > HOLD_LIMIT) routeFps = HOLD_SPEED;
  else routeFps = 0.0;
}



