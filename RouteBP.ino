float barrelHead = 0.0;
float barrelDist = 0.0;
struct loc seekLoc;
float barrelEdgeAngle = 0.0;

/************************************************************************.
 *  barrels() 
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
 *  barrelOne() Strategy #1
 *              Returns true if endY reached.
 ************************************************************************/
boolean barrelOne(boolean reset) {
  if (reset) {
    barrelOneState = SEEK_BARREL;
    barrelX = currentLoc.x;   // Use arrival X
//    setSonar("lFr");
    sonarFront = sonarFrontKeep = 0.0;
    seekBarrel(true);
    return false;
  }
  
  switch (barrelOneState) {
    case SEEK_BARREL:
      if (currentLoc.y > barrelYEnd) return true;
      seekBarrel(false);
//      plotState = 9;
      break;

    case PLOT_BARREL:
      plotBarrel(false);
//      seekState = 9;
      break;

    case CIRCLE_BARREL:
      circleBarrel(false);
//      plotState = 9;
//      seekState = 9;
      break;

    case RECOVER_BARREL:
      recover(false);
//      plotState = 9;
//      seekState = 9;
      break;

    default:
      break;
  }

  return (false);
}



/************************************************************************.
 *  seekBarrel()  Move forward looking for a barrel.
 ************************************************************************/
void seekBarrel(boolean reset) {
  static int endTime = 0;
  if (reset) {
    barrelOneState = SEEK_BARREL;
    seekState = SEEK_SETTLE;
    settle(true); // reset
    endTime = timeMilliseconds + 20000; // 20 seconds before recovery
    isStuck(true);
  } else {
    switch(seekState) {
      case SEEK_SETTLE:
        if (settle(false)) seekState = SEEK_FWD;
        break;

      case SEEK_FWD:
        if (sonarFront < 2.5) {
          plotBarrel(true);  //reset
        } else {
          targetLoc.x = currentLoc.x;
          targetLoc.y = currentLoc.y + 2.0;
          setTarget();
          steerTarget();
        }
        break;

      default:
        break;
    }
//    if (timeMilliseconds > endTime) recover(true);
    if (isStuck(false)) recover(true);
  }
}


const float PIVOT_RATE = 0.5;
const float HEADING_LIMIT = 45.0;
const int PIVOT_LIMIT = 3;
const float SONAR_EDGE = 4.0;
/************************************************************************.
 *  plotBarrel()  A barrel has been detected.  Look left and right.
 ************************************************************************/
void plotBarrel(boolean reset) {
  static int startTime = 0;
  static int pivotCount = 0;
  static int endTime = 0;

//    addLog(
//      (long) (timeMilliseconds),
//      (short) (currentLoc.x * 100.0),
//      (short) (currentLoc.y * 100.0),
//      (short) (gyroHeading * 100.0),
//      (short) (targetWFps * 100.0),
//      (short) (tpFps * 100.0),
//      (short) (routeStepPtr + (barrelOneState * 100) + (seekState * 1000) + (plotState * 10000))
//    );

  if (reset) {
    barrelOneState = PLOT_BARREL;
    plotState = PLOT_SETTLE;
    endTime = timeMilliseconds + 15000; // End seconds.
    pivotCount = 0;
    startTime = timeMilliseconds;
    settle(true);  // reset
    return;
  } else {

    switch (plotState) {
      case PLOT_SETTLE: 
        if (settle(false)) {
          plotState = (currentLoc.x < barrelXCenter) ? PLOT_RIGHT : PLOT_LEFT;
        }
        break;
      case PLOT_RIGHT:
        targetWFpsRight = targetWFps - PIVOT_RATE;
        targetWFpsLeft = targetWFps + PIVOT_RATE;
        if (gHeading > 10.0) {
          if (sonarFront > SONAR_EDGE) {
            barrelEdgeAngle = gHeading;        
            circleBarrel(true); // reset
          }
        } 
        if ((pivotCount > PIVOT_LIMIT) && (gHeading > 0.0)) recover(true);
        if (gHeading > HEADING_LIMIT) {
          pivotCount++;
          plotState = PLOT_LEFT;
        }
        break;
      case PLOT_LEFT:
        targetWFpsRight = targetWFps + PIVOT_RATE;
        targetWFpsLeft = targetWFps - PIVOT_RATE;
        if (gHeading < 10.0) {
          if (sonarFront > SONAR_EDGE) {
            barrelEdgeAngle = gHeading;
            circleBarrel(true); // reset
          }
        } 
        if ((pivotCount > PIVOT_LIMIT) && (gHeading < 0.0)) recover(true);
        if (gHeading < -HEADING_LIMIT) {
          pivotCount++;
          plotState = PLOT_RIGHT;
        }
        break;

      default:
        break;
    }
    if (timeMilliseconds > endTime) recover(true);
  }
}



const float CIRCLE_ORIENT_RATE = 0.2;
const float END_TAN_DEGREES = 50.0;
const float TURN_RADIUS = 2.4;
const float PIVOT_TARGET_X = 1.55;
/************************************************************************.
 *  circleBarrel()
 ************************************************************************/
void circleBarrel(boolean reset) {
  const float DISPLACEMENT_Y = 3.5;
//  static int endTime = 0;
  float displacement;

//    addLog(
//      (long) (timeMilliseconds),
//      (short) (currentLoc.x * 100.0),
//      (short) (currentLoc.y * 100.0),
//      (short) (gyroHeading * 100.0),
//      (short) (targetWFps * 100.0),
//      (short) (tpFps * 100.0),
//      (short) (routeStepPtr + (barrelOneState * 100) + (seekState * 1000) + (plotState * 10000))
//    );

  if (reset) { // set isRightTurn, turnRadius, pivotLoc, targetLoc
    barrelOneState = CIRCLE_BARREL;
//    endTime = timeMilliseconds + 7000;  // Seconds to finish.
    isStuck(true);
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
    } else {
      turn();
//      if (timeMilliseconds > endTime) recover(true);
      if (isStuck(false)) recover(true);
    }
  }
}




const float PED_SONAR_LIMIT = 8.0;
const float PED_ANGLE = 25.0;
/************************************************************************
 *  pedestrian()
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
//    setSonar("lFr");
    sonarFront = sonarFrontKeep = 0.0;
  } else {
    if (sonarFront > 0.1) {
      if (sonarFront < PED_SONAR_LIMIT) {
        sonarPedCount++;
        sonarFront = 0.0;
      }
    }
    if (pedPivotState == PLOT_LEFT) {
      if ((pivotCount > 10) && (gHeading < 0.0)) return true;
      if (gHeading < -PED_ANGLE) {
        if ((sonarPedCount == 0) && pivotCount) return true;
        sonarPedCount = 0;
        pedPivotState = PLOT_RIGHT;
        pivotCount++;
      }
      targetWFpsRight = targetWFps + PIVOT_RATE;
      targetWFpsLeft = targetWFps - PIVOT_RATE;
    } else {
      if (gHeading > PED_ANGLE) {
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



//const float K_HOLD_P = 5.0;
//const float K_HOLD_D = 8.0;
//const float K_HOLD_LIM = 1.88;
const float K_HOLD_P = 2.0;
const float K_HOLD_D = 5.0;
const float K_HOLD_LIM = 1.5;
/************************************************************************.
    holdY() Return fps to seek to y value of seekLoc.
 ************************************************************************/
float holdY() {
  float yError = seekLoc.y - currentLoc.y;
//  if (yError < (-HOLD_LIMIT * 2.0)) routeFps = -HOLD_SPEED * 3.0;
//  else if (yError > (HOLD_LIMIT * 2.0)) routeFps = HOLD_SPEED * 3.0;
//  else if (yError < -HOLD_LIMIT) routeFps = -HOLD_SPEED;
//  else if (yError > HOLD_LIMIT) routeFps = HOLD_SPEED;
//  else routeFps = 0.0;

  float p = yError * K_HOLD_P;
  float d = tpFps * K_HOLD_D;
  return constrain(p - d, -K_HOLD_LIM, K_HOLD_LIM);
}


/************************************************************************.
 *  Jump() 
 ************************************************************************/
void jump(boolean reset) {
  static boolean isJumped = false;
  static int jumpCount = 0;
  static unsigned int jumpTime = 0;

  if (reset) {
    isJumped = false;
    jumpCount = 0;
    jumpTicksEnd = tickPosition + (8.0 * TICKS_PER_FOOT);  // 8 feet
  } else {
    if (!isJumped) {
      if (lsm6.a.z < 13000) {
        jumpCount++;
        if (jumpCount > 20) {
          isJumped = true;
          jumpTime = timeMilliseconds;
        }
      } else {
        jumpCount = 0;
      }
    }

    unsigned int t = timeMilliseconds - jumpTime;
    if (isJumped && (t < 500)) {
      targetWFps = targetWFpsRight = targetWFpsLeft = jumpCompFps; // Can do because in air.
    } else {
      steerHeading();
    } 
  }
//    addLog(
//        (long) (timeMilliseconds),
//        (short) (gaPitch * 100.0),
//        (short) (targetWFps * 100.0),
//        (short) (wFpsRight * 100.0),
//        (short) (wFpsLeft * 100.0),
//        (short) (isJumped),
//        (short) (lsm6.a.z)
//   );

}



/************************************************************************.
 *  settle() Turn toward North.  Return true if pointing North
 *           and is at seek location.
 ************************************************************************/
boolean settle(boolean reset) {
  static unsigned int endTime = 0;
  if (reset) {
    endTime = timeMilliseconds + 3000; // Stop after 3 seconds. 
    seekLoc = currentLoc;
  } else {
    float dy = seekLoc.y - currentLoc.y;
    if (((abs(dy) < 0.1) && (abs(gHeading) < 2.0) && (abs(tpFps) < 0.3))
        || (timeMilliseconds > endTime)) {
//      if (timeMilliseconds > (8000)) {
      return true;
    } else {
      float speedAdjustment = gHeading * 0.05;
      speedAdjustment = constrain(speedAdjustment, -PIVOT_RATE, PIVOT_RATE);
      targetWFpsRight = targetWFps + speedAdjustment;
      targetWFpsLeft = targetWFps - speedAdjustment;
    }
  }
  return false;
}



/************************************************************************.
 *  recover() Back up to somewhere & start over.
 ************************************************************************/
void recover(boolean reset) {
  static boolean isCenterLeft = false;  // True or false depending on location at start,
  static boolean isCentered = false;   
  static unsigned int centerTime = 0;
  static unsigned int endTime = 0;
  float adjustment = 0.0;
  if (reset) {
    endTime = timeMilliseconds + 500;
    barrelOneState = RECOVER_BARREL;
    endTime = timeMilliseconds + 1500;
    centerTime = timeMilliseconds + 500;
    isCentered = false;
    isCenterLeft = (gHeading > 0.0) ? true : false;
    if (currentLoc.x > (barrelXCenter + 2.5)) isBackLeft = true;
    else if (currentLoc.x < (barrelXCenter - 2.5)) isBackLeft = false;
  } else {
    if (timeMilliseconds > endTime) {                          // Done moving?
      barrels(true); // reset
    } else if ((currentLoc.x > (barrelXCenter + 2.5)) && !isBackLeft) {        // Reached right edge?
      isBackLeft = true; // Set for next iteration.
      barrels(true); // reset 
    } else if ((currentLoc.x < (barrelXCenter - 2.5)) && isBackLeft)  {        // Reached left edge?
      isBackLeft = false; // Set for next iteration.
      barrels(true); // reset 
    } else if ((timeMilliseconds < centerTime) || !isCentered) {  // Still centering?
      if (isCenterLeft && (gHeading <  10.0)) {
        isCentered = true;
      } else if (!isCenterLeft && (gHeading > -10.0)) {
        isCentered = true;
      } else {
        adjustment = PIVOT_RATE * 2.0;
        if (!isCenterLeft) adjustment = -adjustment;
      }
    } else {                                                     // Center 
      adjustment = PIVOT_RATE * 0.7;
      if (isBackLeft) adjustment = -adjustment;
    }
    targetWFpsRight = targetWFps + adjustment;
    targetWFpsLeft = targetWFps - adjustment; 
  }
}


#define MOVE_THRESH 0.1   // min feet to move each second
/************************************************************************.
 *  isStuck() Return true if hasn't moved in last second.
 ************************************************************************/
boolean isStuck(boolean reset) {
  static unsigned int startTime;
  static unsigned int startTicks;
  static unsigned int lastSeconds;

  if (reset) {
    startTime = timeMilliseconds;
    startTicks = tickPosition;
    lastSeconds = 0;
  } else {
    unsigned int seconds = (timeMilliseconds - startTime) / 1000;
//        addLog(
//          (long) (timeMilliseconds),
//          (short) (seconds),
//          (short) (lastSeconds),
//          (short) (startTime),
//          (short) (tickPosition),
//          (short) (startTicks),
//          (short) (routeStepPtr + (barrelOneState * 100) + (seekState * 1000) + (plotState * 10000))
//        );
    if (seconds > lastSeconds) {
      int distance = tickPosition - startTicks;
      if ((distance < (MOVE_THRESH * TICKS_PER_FOOT)) && (seconds > 3)) {
        return true;
      }
      startTicks = tickPosition;
      lastSeconds = seconds;
    }
  }
  return false;
}
 

