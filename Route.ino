#define STEP_ERROR -42.42
int originalStepStringPtr = 0;
struct loc savedOrientationLoc;
struct loc savedPositionLoc;
String stepString = "";
int numLen = 0;
int loopX = 0;
boolean isSaveOrientation = false;
boolean isSavePosition = false;
boolean isFixOrientation = false;
long endStandTime = 0L;
double routeStandFps = 0.0D;
float barrelXAxis = 0.0;
char axisC = ' ';
double dDiff = 0.0D;

double degreesPerTick;
double turnStartBearing;
int startTurnTick;
double avgTargetBearing = 0.0;
//double reFactor = 1.0;  // From tests at 3.5fps, 1.5 radius
//double reFactor = 2.0;  // From hous3
//double reFactor = 0.0;  // From hous3
double startOrientation = 0.0;
struct loc startLoc;

void routeLog() {
  if (isRouteInProgress && isRunning) {
    addLog(
      (long) (timeRun),
      (short) (currentLoc.x * 100.0),
      (short) (currentLoc.y * 100.0),
      (short) (gyroHeading * 100.0),
      (short) (sonarFront * 100.0),
      (short) (barrelX * 100.0),
      (short) (routeStepPtr - isDecelPhase)
    );
  }
}

/************************************************************************
    steerRoute() called every loop (208/sec).
            This is called as the last step in aTp6() for steering.
              1. Check if target reached.
              2. Set currentLoc.
              3. Adjust steering.
 ************************************************************************/
void steerRoute() {
  boolean isNewRouteStep = false;
  boolean ret;
  timeRun = timeMilliseconds - timeStart;
  targetWFpsRight = targetWFpsLeft = targetWFps;

  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'C':
    case 'E':
    case 'L':
    case 'M':
    case 'N':
    case 'D':
      isNewRouteStep = true;
      break;

    case 'B':
      if (barrels(false)) isNewRouteStep = true;
      break;

    case 'G':
      setTarget();
//      ret = setFps();
//      if (!isDecelActive) ret = isTargetReached();
//      if (ret) {
//        isDecelPhase = isDecelActive = false;
//        isNewRouteStep = true;
//      } else {
        steerHeading();
//      }
      break;

    case 'K':  // Lock
      if (isStartReceived) {
        isNewRouteStep = true;
        setLockDrift();
        setHeading(rangeAngle(startOrientation));
        currentLoc = startLoc;
        timeStart = timeMilliseconds;
        routeFps = 0.0;
        isRunReady = true;
      }
      else {
        routeFps = 0.0;
        addLockHeading();
      }
      break;

    case 'P':
      if (pedestrian(false)) {
        isNewRouteStep = true;
      }
      break;

    case 'T': // Turn
      setTarget();
      ret = setFps();
      if (!isDecelActive) ret = isTargetReached();
      if (ret) {
        isDecelPhase = isDecelActive = false;
        isNewRouteStep = true; 
      } else {
        turn();
      }
      break;

    default:
      isRouteInProgress = false;
      sprintf(message, "Illegal step: %d", routeStepPtr);
      sendBMsg(SEND_MESSAGE, message);
      sendXMsg(SEND_MESSAGE, message);
      break;
  } // end switch()

  // Move to new action if current action is done.
  if (isNewRouteStep) { // Move to next action in list.
    interpretRouteLine(getNextStepString());
    sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction);
    sendBMsg(SEND_MESSAGE, message);
    sendXMsg(SEND_MESSAGE, message);

  }
}



/************************************************************************
    interpretRouteLine()
        Called every time the end criterion for a route step is reached.
        Read the new route step and set the values.
 ************************************************************************/
boolean interpretRouteLine(String ss) {
  double pivotDir, dblVal, aDiff;
  int retInt;
  char char1, char2;
  struct chartedObject co;

  stepString = ss;
  isSaveOrientation = isSavePosition = isFixOrientation = false;
  originalStepStringPtr = 0;
  Serial.print(stepString);  Serial.print(":   ");
  routeCurrentAction = stepString.charAt(0);
  stepString = stepString.substring(1);
  originalStepStringPtr++;

  (*currentValSet).v = 2.0;  // restore speed correction.

  switch (routeCurrentAction) {
    case 'B': // Barrels
      barrelXCenter = readNum();
      Serial.print(barrelXCenter); Serial.print("   ");
      if (barrelXCenter == STEP_ERROR) return false;
      barrelYEnd = readNum();
      Serial.print(barrelYEnd); Serial.print("   ");
      if (barrelYEnd == STEP_ERROR) return false;
      setSonar("lFr");
      barrels(true); // Reset
      break;

    case 'C': // Charted object
      char1 = stepString.charAt(0);
      if (char1 == 'R') co.isRightSonar = true;
      else if (char1 == 'L') co.isRightSonar = false;
      else return false;
      stepString = stepString.substring(1);
      co.trigger = readNum();
      Serial.print(co.trigger); Serial.print("   ");
      if (co.trigger == STEP_ERROR) return false;
      co.surface = readNum();
      Serial.print(co.surface); Serial.print("   ");
      if (co.surface == STEP_ERROR) return false;
      chartedObjects[coEnd++] = co;
      break;

    case 'D':  // Decelerate
      decelFps = readNum();
      Serial.print(decelFps); Serial.print("   ");
      if (decelFps == STEP_ERROR) return false;
      isDecelActive = true;
      break;

    case 'F':  // Fini
      isRouteInProgress = false;
      break;

    case 'G':  // Go to the next waypoint
      lsPtr = 0;
      isHug = false;
      targetLoc = readLoc();
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      routeFps = routeScriptFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      break;

    case 'K': // Lock bearing at start
      char1 = stepString.charAt(0);
      if (char1 == 'S') isLockStand = true;
      else if (char1 == 'R') isLockStand = false;
      else return false;
      stepString = stepString.substring(1);
      startLoc = readLoc();
      Serial.print(startLoc.x); Serial.print("  "); Serial.print(startLoc.y); Serial.print("   ");
      if (startLoc.y == STEP_ERROR) return false;
      startOrientation = readNum();
      Serial.print(startOrientation); Serial.print("   ");
      if ((startOrientation < -180.0D) || (startOrientation > 180.0)) return false;
      resetLockHeading();
      isStartReceived = false;
      break;

    case 'M':
      char1 = stepString.charAt(0);
      if      (char1 == 'R') setSonar("lfR");
      else if (char1 == 'F') setSonar("lFr");
      else if (char1 == 'L') setSonar("Lfr");
      else if (char1 == 'A') setSonar("LFR");
      else if (char1 == 'N') setSonar("lfr");
      else return false;
      break;

    case 'N':  // Name of route
      stripWhite();
      routeTitle = stepString;
      break;

    case 'P':  // Pedestrian, place holder
      pedestrian(true);
      break;

    case 'T':  // Turn at radius ending at waypoint.
      targetLoc = readLoc();
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      setTarget();
      aDiff = rangeAngle(targetBearing - gyroHeading);
      isRightTurn = aDiff > 0.0;

      routeFps = routeScriptFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      turnRadius = readNum();
      Serial.print(turnRadius);  Serial.print("   ");
      if (turnRadius == STEP_ERROR) return false;
      endTangentDegrees = readNum();
      Serial.print(endTangentDegrees);  Serial.print("   ");
      if (endTangentDegrees == STEP_ERROR) return false;

      pivotBearing = endTangentDegrees + (isRightTurn ?  90.0 :  -90.0);
      pivotBearing = rangeAngle(pivotBearing);
      //      if (isRightTurn) {
      pivotLoc.x = targetLoc.x + (sin(pivotBearing * DEG_TO_RAD) * turnRadius);
      pivotLoc.y = targetLoc.y + (cos(pivotBearing * DEG_TO_RAD) * turnRadius);
      //      } else {
      //        pivotLoc.x = targetLoc.x + (sin(pivotBearing * DEG_TO_RAD) * turnRadius);
      //        pivotLoc.y = targetLoc.y - (cos(pivotBearing * DEG_TO_RAD) * turnRadius);
      //      }
      Serial.println();
      sprintf(message, "       turn = %s ", (isRightTurn ? "R" : "L"));
      sendBMsg(SEND_MESSAGE, message);
      Serial.print(message);
      sprintf(message, "      pivotLoc.: %5.2f,%5.2f", pivotLoc.x, pivotLoc.y);
      sendBMsg(SEND_MESSAGE, message);
      Serial.print(message);
      break;

    default:
      Serial.println("Step error: Illegal command.");
      return false;
  }
  Serial.println();
  //  sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction); isNewMessage = true;
  return true;
}




//#define RAD_TURN 10.0
#define RAD_TURN 4.0
#define END_STEER 0.5
/************************************************************************
    steerHeading() Find the correct heading to the target and adjust the
                 wheel speeds to turn toward the target.  As tp approaches
                 the target, use the originalTargetBearing.
 ************************************************************************/
void steerHeading() {
  double speedAdjustment;

  if (targetDistance < 1.0) {
    speedAdjustment = 0.0;
  } else {
    double aDiff = rangeAngle(targetBearing - gyroHeading);
    double d = (aDiff > 0.0) ? 1.0 : -1.0;

    speedAdjustment = (wFps / RAD_TURN) * 0.64 * d;

    // Reduce adjustment proportionally if less than X degrees.
    if (abs(aDiff) < 5.0) {
      speedAdjustment = (abs(aDiff) / 5.0) * speedAdjustment;
    }
  }
  targetWFpsRight = targetWFps - speedAdjustment;
  targetWFpsLeft = targetWFps + speedAdjustment;
}



/***********************************************************************.
    turn() Turn with a given radius.
 ************************************************************************/
void turn() {
  float speedAdjustment, headingError;
  float xDist, yDist, radiusAngle, targetTurnHeading;

  float d = isRightTurn ? 1.0 : -1.0;
  float radiusDiff = (wFps / turnRadius) * 0.54 * d;

  if (isRightTurn) {
    xDist = (currentLoc.x - pivotLoc.x);
    yDist = (currentLoc.y - pivotLoc.y);
    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
    targetTurnHeading = rangeAngle(radiusAngle + 90.0);
    headingError = rangeAngle(gyroHeading - targetTurnHeading);
  } else {
    xDist = (currentLoc.x - pivotLoc.x);
    yDist = (currentLoc.y - pivotLoc.y);
    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
    targetTurnHeading = rangeAngle(radiusAngle - 90.0);
    headingError = rangeAngle(gyroHeading - targetTurnHeading);
  }

  float radiusError = sqrt((xDist * xDist) + (yDist * yDist)) - turnRadius;
  float radiusAdjustment = radiusError * 0.8 * d;
  float headingAdjustment = -headingError * 0.03;
//  float radiusAdjustment = radiusError * 0.2 * d;
//  float headingAdjustment = -headingError * 0.02;
  speedAdjustment = radiusDiff + headingAdjustment + radiusAdjustment;
//  speedAdjustment = radiusDiff;

  targetWFpsRight = targetWFps - speedAdjustment;
  targetWFpsLeft = targetWFps + speedAdjustment;
//    addLog(
//    (long) (timeMilliseconds),
//    (short) (currentLoc.x * 100.0),
//    (short) (currentLoc.y * 100.0),
//    (short) (targetDistance * 100.0),
//    (short) (gyroHeading * 100.0),
//    (short) (speedAdjustment * 100.0),
//    (short) (sonarFront)
//  );
}

/************************************************************************
    setFps() Limit acceleration to prevent too rapid acceleration--
             particularly when starting from zero from the stand.
             Also starts deceleration at end of step.
             Returns true if is decelerating and reached target fps.
 ************************************************************************/
boolean setFps() {
//  float stopDistance;
//
//  // Check to see if we need to start the decel phase.
//  if (isDecelActive && !isDecelPhase) {
//    stopDistance = (fps * fps) / 5.58;
//    if (targetDistance <= stopDistance) {
//      isDecelPhase = true;
//    }
//  }
//
//  if (isDecelPhase) {
//    if (fps <= 0.0) return true;
//    routeFps = fps - 5.0;
//  } else {
//    if (routeScriptFps < 0.0) {
//      routeFps = routeScriptFps;
//    } else { // Going forward
//      if (timeRun < 1000) {  // taking off from stand?
//        routeFps = (((float) timeRun) / 200.0) + 0.5;
//      } else {
//        routeFps = fps + 5.0;
//        if (routeFps > (routeScriptFps + 2.5)) {
//          float inc = (routeScriptFps - fps) / 2.0;
//          routeFps = routeScriptFps + inc;
//        }
//      }
//    }
//  }
  routeFps = routeScriptFps;
  return false;
}



/************************************************************************
    setTarget() Set the new targetBearing and targetDistance from
                the currentLoc.
 ************************************************************************/
void setTarget() {
  double x =  targetLoc.x - currentLoc.x;
  double y = targetLoc.y - currentLoc.y;
  targetBearing = atan2(x, y) * RAD_TO_DEG;
  double xTargetDist = currentLoc.x - targetLoc.x;
  double yTargetDist = currentLoc.y - targetLoc.y;
  targetDistance = sqrt((xTargetDist * xTargetDist) + (yTargetDist * yTargetDist));
}



/***********************************************************************.
    isTargetReached()  Return true if within 1 ft of target and is
                       moving away from the target.
 ***********************************************************************/
boolean isTargetReached() {
  const int RETREAT_TIMES = 10;
  const int RETREAT_DISTANCE = 2.0;
  static double lastX = 0.0D;
  static double lastY = 0.0D;
  static double lastTargetDist = 10000.0D;
  static int timesReached = 0;

  if (targetDistance < RETREAT_DISTANCE) {
    boolean isCloser = ((lastTargetDist - targetDistance) >= 0.0D);
    lastTargetDist = targetDistance;
    lastX = currentLoc.x;
    lastY = currentLoc.y;
    if (isCloser) { // Getting closer?
      timesReached = 0;
    } else {
      timesReached++;
      // Return true after Nth time.
      if (timesReached > RETREAT_TIMES) {
        timesReached = 0;
        return true;
      }
    }
  } else {
    timesReached = 0;
  }
  return false;
}


/***********************************************************************.
    Script parsing routines
 ***********************************************************************/
double readNum() {
  stripWhite();
  double num = stepString.toFloat();
  stripNum();
  if (numLen == 0) return STEP_ERROR;
  return num;
}

struct loc readLoc() {
  struct loc locLoc = {STEP_ERROR, STEP_ERROR};
  stripWhite();
  locLoc.x = stepString.toFloat();
  stripNum();
  if (numLen == 0) return locLoc;
  if (stepString.charAt(0) != ',') return locLoc;
  stepString = stepString.substring(1);
  //  stripWhite();
  double y = stepString.toFloat();
  stripNum();
  if (numLen == 0) return locLoc;
  locLoc.y = y;
  return locLoc;
}

void stripWhite() {
  int ptr = 0;
  int len = stepString.length();
  while (true) {
    if (ptr >= len) break;
    char c = stepString.charAt(ptr);
    if ((c != ' ') && (c != '\t')) break;
    ptr++;
  }
  originalStepStringPtr += ptr;
  stepString = stepString.substring(ptr);
}


void stripNum() {
  int ptr = 0;
  int len = stepString.length();
  while (true) {
    if (ptr >= len) break;
    char c = stepString.charAt(ptr);
    if ((c == '.') || (c == '-') || ((c >= '0') && (c <= '9'))) ptr++;
    else break;
  }
  originalStepStringPtr += ptr;
  stepString = stepString.substring(ptr);
  numLen = ptr;
}

char readChar() {
  stripWhite();
  if (stepString.length() == 0) return 0;
  char c = stepString.charAt(0);
  stepString = stepString.substring(1);
  return c;
}



#define LOCK_SIZE 416
float zLockSum = 0.0;
float xLockSum = 0.0;
int gLockCount = 0;
/***********************************************************************.
    ???LockHeading()  Average drift in one-second blocks while
                      in a startup lock condition.
 ***********************************************************************/
void addLockHeading() {
  static int zArray[LOCK_SIZE];
  static int xArray[LOCK_SIZE];
  static int gPtr = 0;
  int z, x, zMin, zMax, xMin, xMax;
  float zAve, xAve;

  zArray[gPtr] = lsm6.g.z;
  xArray[gPtr] = lsm6.g.x;
  gPtr++;
  if (gPtr >= LOCK_SIZE) {
    int zSum = 0;
    int xSum = 0;
    zMax = zMin = zArray[0];
    xMax = xMin = xArray[0];
    for (int i = 0; i < LOCK_SIZE; i++) {
      z = zArray[i];
      if (z > zMax)  zMax = z;
      if (z < zMin)  zMin = z;
      zSum += z;
      x = xArray[i];
      if (x > xMax)  xMax = x;
      if (x < xMin)  xMin = x;
      xSum += x;
    }
    zAve = ((float) zSum) / ((float) LOCK_SIZE);
    xAve = ((float) xSum) / ((float) LOCK_SIZE);
    if (((zMax - zMin) < 90) && ((xMax - xMin) < 90)) {
      zLockSum += zAve;
      xLockSum += xAve;
      gLockCount++;
    }
    sprintf(message, "xMin: %4d     xMax: %4d     xAve: %5.1f     ", xMax, xMin, xAve);
    Serial.print(message);
    sprintf(message, "zMin: %4d     zMax: %4d     zAve: %5.1f\n", zMax, zMin, zAve);
    Serial.print(message);
    gPtr = 0;
  }
}
void resetLockHeading() {
  timeDriftYaw = 0.0;
  gLockCount = 0;
  zLockSum = 0.0;
  xLockSum = 0.0;
  lockStartTicks = tickPosition;
}
void setLockDrift() {
  timeDriftYaw = zLockSum / ((float) gLockCount);
  timeDriftPitch = xLockSum / ((float) gLockCount);
  sprintf(message, "YawDrift: %5.2f     PitchDrift: %5.2f\n", timeDriftYaw, timeDriftPitch);
  Serial.print(message);
}



