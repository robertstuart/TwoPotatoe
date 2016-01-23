
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
char axisC = ' ';
double dDiff = 0.0D;

double degreesPerTick;
double turnStartBearing;
int startTurnTick;
//double reFactor = 1.0;  // From tests at 3.5fps, 1.5 radius
//double reFactor = 2.0;  // From hous3
double reFactor = 0.0;  // From hous3

void routeLog() {
  addLog(
    (long) (timeMilliseconds),
    (short) (currentMapLoc.x * 100.0),
    (short) (currentMapLoc.y * 100.0),
    (short) (sonarLeft * 100.0),
    (short) (sonarRight * 100.0),
    (short) (gyroHeading * 100.0),
    (short) routeStepPtr
  );
}

/************************************************************************
 *  Route() called every loop (400/sec).
 *          This is called as the last step in aTp6() for steering.
 *            1. Check if target reached.
 *            2. Set currentLoc.
 *            3. Adjust steering.
 ************************************************************************/
void route() {
  boolean isNewRouteStep = false;
  setTargetBearing();
  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'B':
    case 'C':
    case 'E':
    case 'H':
    case 'L':
    case 'N':
    case 'W':
    case 'Z':
      isNewRouteStep = true;
      break;

    case 'A':  // Adjust position and bearing
      if (isEndStand()) {
        isNewRouteStep = true;
        //        if (isMagAdjust) setHeading(gmHeading);
        if (isMagAdjust) setHeading(tmHeading);
        else setHeading(0.0D);
        //        setHeading(0.0D);
        currentMapLoc.x = 0.0D;
        currentMapLoc.y = 0.0D;
      }
      else adjustPosition();
      break;

    case 'D':  // Discombobulator, going south.
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      if (routeTargetXYDistance >= 0.0) isNewRouteStep = true;
      discombobulate();
      break;

    case 'G':
      if (coPtr < coEnd) doChartedObject();
      if (isReachedTrigger(routeTargetXY)) {
        isNewRouteStep = true;
        coPtr = 0;
        coEnd = 0;
      }
      steerHeading();
      break;

    case 'K':  // Discombobulator, going north.
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      if (routeTargetXYDistance <= 0.0) isNewRouteStep = true;
      discombobulate();
      break;

    case 'P':
      if (pirouette()) isNewRouteStep = true;
      break;

    case 'S': // Stop. Next step when WS, Pitch, Bearing, COS are near zero.
      if (stopFix()) isNewRouteStep = true;
      break;

    case 'T': // Turn
      if (isTurnDegrees) {
        if (routeIsRightTurn) {
          if (currentMapCumHeading > turnTargetCumHeading) isNewRouteStep = true;
        } else {
          if (currentMapCumHeading < turnTargetCumHeading) isNewRouteStep = true;
        }
      }
      else {
        if (abs(routeTargetBearing - currentMapHeading) < 5.0)  isNewRouteStep = true;
      }
      turnRadius();
      break;

    case 'U': // Hug
      if (isReachedTrigger(routeTargetXY)) isNewRouteStep = true;
      steerHug();
      break;

    default:
      isRouteInProgress = false;
      sprintf(message, "Illegal step: %d", routeStepPtr); isNewMessage = true;
      break;
  } // end switch()

  // Move to new action if current action is done.
  if (isNewRouteStep) { // Move to next action in list.
    interpretRouteLine(getNextStepString());
    sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction); isNewMessage = true;
  }
}



/************************************************************************
 *  interpretRouteLine()
 *      Called every time the end criterion for a route step is reached.
 *      Read the new route step and set the values.
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

    case 'A': // Adjust position and bearing.
      isMagAdjust = false;
      char1 = stepString.charAt(0);
      if (char1 == 'M') {
        isMagAdjust = true;
        stepString = stepString.substring(1);
        gridOffset = readNum();
        Serial.print(gridOffset); Serial.print("   ");
        if ((gridOffset < -180.0D) || (gridOffset > 180.0)) return false;
        setHeading(rangeAngle(magHeading - gridOffset));
      }
      fixPosition = tickPosition;
      fixHeading = tickHeading;
      routeTargetLoc.y = 100.0; // For following command.
      routeTargetLoc.x = 0.0;
      //      setPhantomTarget();
      break;

    case 'C': // Charted object
      char1 = stepString.charAt(0);
      if (char1 == 'R') co.isRightSonar = true;
      else if (char1 == 'L') co.isRightSonar = false;
      else return false;
      stepString = stepString.substring(1);
      char2 = stepString.charAt(0);
      co.type = ' ';
      if (!isspace(char2)) {
        co.type = char2;
        if ((char2 != 'S') && (char2 != 'H')) return false;
        stepString = stepString.substring(1);
      }
      co.trigger = readNum();
      Serial.print(co.trigger); Serial.print("   ");
      if (co.trigger == STEP_ERROR) return false;
      co.surface = readNum();
      Serial.print(co.surface); Serial.print("   ");
      if (co.surface == STEP_ERROR) return false;
      chartedObjects[coEnd++] = co;
      break;

    case 'D':  // Discombobulator
    case 'K':  // Discombobulator
      routeTargetLoc = readLoc();
      Serial.print(routeTargetLoc.x); Serial.print("  "); Serial.print(routeTargetLoc.y); Serial.print("   ");
      if (routeTargetLoc.y == STEP_ERROR) return false;
      routeTargetXY = routeTargetLoc.y;

      routeFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      break;

    case 'E':
      break;

    case 'F':
      isRouteInProgress = false;
      //reFactor += .2;
      break;

    case 'G':
      lsPtr = 0;
      isHug = false;
      char1 = stepString.charAt(0);
      if (char1 == 'X') isGXAxis = true;
      else if (char1 == 'Y') isGXAxis = false;
      else return false;
      stepString = stepString.substring(1);
      char2 = stepString.charAt(0);
      if (!isspace(char2)) {
        if (char2 == 'R') isRightSonar = true;
        else if (char2 == 'L') isRightSonar = false;
        else return false;
        stepString = stepString.substring(1);
      }

      routeTargetLoc = readLoc();
      Serial.print(routeTargetLoc.x); Serial.print("  "); Serial.print(routeTargetLoc.y); Serial.print("   ");
      if (routeTargetLoc.y == STEP_ERROR) return false;
      if (isGXAxis) {
        routeTargetXY = routeTargetLoc.x;
        if (currentMapLoc.x < routeTargetLoc.x) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      else {
        routeTargetXY = routeTargetLoc.y;
        if (currentMapLoc.y < routeTargetLoc.y) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }

      routeFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;

      routeRadius = readNum();
      Serial.print(routeRadius);  Serial.print("   ");
      if (routeRadius < .5) routeRadius = 0.5;
      if (routeRadius == STEP_ERROR) routeRadius = 2.0;
      break;

    case 'H':
      if (headingSource == HEADING_SOURCE_M) {
        gyroCumHeading = gyroHeading = gridHeading;
        tickCumHeading = tickHeading = gridHeading;
        gmCumHeading = gmHeading = gridHeading;
        tmCumHeading = tmHeading = gridHeading;
      }
      char1 = stepString.charAt(0);
      if      (char1 == 'G')  headingSource = HEADING_SOURCE_G;
      else if (char1 == 'M') headingSource = HEADING_SOURCE_M;
      else if (char1 == 'T') headingSource = HEADING_SOURCE_T;
      else if (char1 == 'H') headingSource = HEADING_SOURCE_GM;
      break;

    case 'L':
      //      lsBearing = readNum();
      //      Serial.print(lsBearing);  Serial.print("   ");
      //      if (lsBearing == STEP_ERROR) return false;
      lsXYSurface = readNum();
      Serial.print(lsXYSurface);  Serial.print("   ");
      if (lsXYSurface == STEP_ERROR) return false;
      leastSquares();
      break;

    case 'N':
      stripWhite();
      routeTitle = stepString;
      break;

    case 'P':
      dblVal = readNum();
      Serial.print(dblVal); Serial.print("  ");
      if (dblVal == STEP_ERROR) return false;
      routeIsRightTurn = (dblVal >= 0) ? true : false;
      turnTargetCumHeading = currentMapCumHeading + dblVal;
      dblVal = readNum();
      Serial.print(dblVal); Serial.print("  ");
      if (dblVal == STEP_ERROR) return false;
      pirouetteFps = dblVal;
      routeFps = 0.9D;
      break;

    case 'R':
      routeStepPtr = (int) readNum();
      Serial.print(retInt);
      if ((routeStepPtr == STEP_ERROR) || (routeStepPtr >= (routeStepPtr - 2))) return false;
      break;

    case 'S':  // Stop
      char1 = stepString.charAt(0);
      stepString = stepString.substring(1);
      char2 = stepString.charAt(0);
      if (char2 == 'W') {
        isRouteWait = true;
        stepString = stepString.substring(1);
      }
      else {
        isRouteWait = false;
      }

      dblVal = readNum();
      Serial.print(dblVal); Serial.print("   ");
      if (dblVal == STEP_ERROR) return false;
      switch (char1) {
        case 'X':
          axisC = 'X';
          routeTargetLoc.x = dblVal;
          break;
        case 'x':
          axisC = 'X';
          routeTargetLoc.x = jumpFallXY + dblVal;
          break;
        case 'Y':
          axisC = 'Y';
          routeTargetLoc.y = dblVal;
          break;
        case 'y':
          axisC = 'Y';
          routeTargetLoc.y = jumpFallXY + dblVal;
          break;
        default:
          return false;
      }
      if (axisC == 'X') {
        if (routeTargetLoc.x < routeTargetLoc.x) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      else {
        routeTargetXY = routeTargetLoc.y;
        if (routeTargetLoc.y < routeTargetLoc.y) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      break;

    case 'T':  // Turn
      char1 = stepString.charAt(0);
      if ((char1 == 'R') || (char1 == 'L')) {
        isTurnDegrees = true;
        routeIsRightTurn = (char1 == 'R') ? true : false;
        stepString = stepString.substring(1);
        dblVal = readNum();
        Serial.print(dblVal); Serial.print("  ");
        if (dblVal == STEP_ERROR) return false;
        turnTargetCumHeading = currentMapCumHeading + dblVal;
      }
      else {
        isTurnDegrees = false;
        routeTargetLoc = readLoc();
        Serial.print(routeTargetLoc.x); Serial.print("  "); Serial.print(routeTargetLoc.y); Serial.print("   ");
        if (routeTargetLoc.y == STEP_ERROR) return false;
        setTargetBearing();
        aDiff = routeTargetBearing - currentMapHeading;
        if (aDiff > 180.0) aDiff -= 360.0;
        else if (aDiff < -180.0) aDiff += 360.0;
        routeIsRightTurn = aDiff > 0.0;
      }
      routeFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      routeRadius = readNum();
      Serial.print(routeRadius);  Serial.print("   ");
      if (routeRadius < 0.5) routeRadius = 0.5;
      if (routeRadius == STEP_ERROR) routeRadius = 2.0D ;
      pivotDir = routeIsRightTurn ? (currentMapHeading + 90.0) : (currentMapHeading - 90.0);
      pivotDir = rangeAngle(pivotDir);
      pivotLoc.x = (sin(pivotDir * DEG_TO_RAD) * routeRadius) +  currentMapLoc.x;
      pivotLoc.y = (cos(pivotDir * DEG_TO_RAD) * routeRadius) +  currentMapLoc.y;
      //      sprintf(message, "pivotLoc.x: %5.2f     pivotLoc.y: %5.2f", pivotLoc.x, pivotLoc.y);
      //      sendBMsg(SEND_MESSAGE, message);
      degreesPerTick = 180.0D / (TICKS_PER_FOOT * PI * routeRadius);
      if (!routeIsRightTurn) degreesPerTick = -degreesPerTick;
      turnStartBearing = currentMapHeading; // Need to fix this..........
      startTurnTick = tickPosition;
      break;

    case 'U':
      lsPtr = 0;
      isHug = true;
      char1 = stepString.charAt(0);
      if (char1 == 'N') hugBearing = 0.0D;
      else if (char1 == 'E') hugBearing = 90.0D;
      else if (char1 == 'S') hugBearing = 180.0D;
      else if (char1 == 'W') hugBearing = -90.0D;
      else return false;
      stepString = stepString.substring(1);
      char2 = stepString.charAt(0);
      if (char2 == 'R') isRightSonar = true;
      else if (char2 == 'L') isRightSonar = false;
      else return false;
      stepString = stepString.substring(1);
      routeTargetXY = readNum();
      Serial.print(routeTargetXY); Serial.print("  ");
      if (routeTargetXY == STEP_ERROR) return false;
      if (isGXAxis) {
        if (currentMapLoc.x < routeTargetXY) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      else {
        if (currentMapLoc.y < routeTargetXY) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      routeFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      hugXYRhumb = readNum();
      Serial.print(hugXYRhumb);  Serial.print("   ");
      if (hugXYRhumb == STEP_ERROR) return false;
      hugXYSurface = readNum();
      Serial.print(hugXYSurface);  Serial.print("   ");
      if (hugXYSurface == STEP_ERROR) return false;
//      hugSonarDistance = abs(hugXYRhumb - hugXYSurface);
      break;

    default:
      Serial.println("Step Error.");
      return false;
  }
  Serial.println();
  //  sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction); isNewMessage = true;
  return true;
}


/************************************************************************
 *  isEndStand() return true if EndStand buttons pressed or message
 *               from controller.
 ************************************************************************/
boolean isEndStand() {
  if (isEsReceived) {
    isEsReceived = false;
    return true;
  }
  if (digitalRead(GN_SW_PIN) == LOW) return true;
  if (digitalRead(RE_SW_PIN) == LOW) return true;
  return false;
}


/************************************************************************
 *  doChartedObject()
 ************************************************************************/
void doChartedObject() {
  struct chartedObject co = chartedObjects[coPtr];
  if (!isReachedTrigger(co.trigger)) return;

  double xyVal;
  loc beforeLoc = currentMapLoc;

  // Compute min value of last  2 feet
  int n = ((int) (20.0D / routeFps)) + 1; // 2 feet of measurement.
  double minVal = 20.0D;
  if (co.isRightSonar) {
    for (int i = 0; i < n; i++) {
      int index = sonarRightArrayPtr - i - 1;
      if (index < 0) index += SONAR_ARRAY_SIZE; //% is not modulus!
      double val = sonarRightArray[index];
      if (val < minVal) minVal = val;
    }
  } else {
    for (int i = 0; i < n; i++) {
      int index = sonarLeftArrayPtr - i - 1;
      if (index < 0) index += SONAR_ARRAY_SIZE; //% is not modulus!
      double val = sonarLeftArray[index];
      if (val < minVal) minVal = val;
    }
  }
//sprintf(message, "Sonar samples: %2d", n);
//sendBMsg(SEND_MESSAGE, message);
// 7 samples at 3fps
  if (!co.isRightSonar) {
    if (isGXAxis) {
      // Left sonar.
      if (isRouteTargetIncreasing) {
        currentMapLoc.y = co.surface - minVal;
      } else {
        currentMapLoc.y = co.surface + minVal;
      }
    } else {
      if (isRouteTargetIncreasing) {
        currentMapLoc.x = co.surface + minVal;
      } else {
        currentMapLoc.x = co.surface - minVal;
      }
    }
  } else {
    // Right sonar.
    if (isGXAxis) {
      if (isRouteTargetIncreasing) {
        currentMapLoc.y = co.surface + minVal;
      } else {
        currentMapLoc.y = co.surface - minVal;
      }
    } else {
      if (isRouteTargetIncreasing) {
        currentMapLoc.x = co.surface - minVal;
      } else {
        currentMapLoc.x = co.surface + minVal;
      }
    }
  }
  
  if (co.type == 'S') {
    coSetLoc = currentMapLoc;
  } else if (co.type == 'H') { // fix heading
    double actualHeading = atan2(currentMapLoc.y - coSetLoc.y,  currentMapLoc.x - coSetLoc.x) * RAD_TO_DEG;
    double tpHeading = atan2(beforeLoc.y - coSetLoc.y, beforeLoc.x - coSetLoc.x) * RAD_TO_DEG;
    double angleCorrection = actualHeading - tpHeading;
    setHeading(currentMapHeading - angleCorrection);
    sprintf(message, "%5.2f \t%5.2f \t%5.2f", angleCorrection, tpHeading, actualHeading);
    sendBMsg(SEND_MESSAGE, message);
  }
  coPtr++;
  sprintf(message, "minVal: %5.2f\t  oldX: %5.2f\t oldY: %5.2f\t newX: %5.2f\t newY: %5.2f", 
  minVal, beforeLoc.x, beforeLoc.y, currentMapLoc.x, currentMapLoc.y);
  sendBMsg(SEND_MESSAGE, message);
//  currentMapLoc = beforeLoc;
}


//#define RAD_TURN 10.0
#define RAD_TURN 4.0
#define END_STEER 0.5
/************************************************************************
 *  steerHeading() Find the correct heading to the target and adjust the
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 ************************************************************************/
void steerHeading() {
  double speedAdjustment;
  if (targetDistance < 1.0) {
    speedAdjustment = 0.0;
  } else {
    double aDiff = routeTargetBearing - currentMapHeading;
    if (aDiff > 180.0) aDiff -= 360.0;
    else if (aDiff < -180.0) aDiff += 360.0;
    double d = (aDiff > 0.0) ? 1.0 : -1.0;

    speedAdjustment = (wheelSpeedFps / RAD_TURN) * 0.64 * d;

    // Reduce adjustment proportionally if less than X degrees.
    if (abs(aDiff) < 5.0) {
      speedAdjustment = (abs(aDiff) / 5.0) * speedAdjustment;
    }
  }
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}

/************************************************************************
 *  steerHug() Follow along a wall at the specified distance.
 *             Do not steer beyong 20 degrees from the specified angle.
 ************************************************************************/
void steerHug() {
  double hugTargetDistance, angleError;

  if (isRightSonar) {
    hugTargetDistance = hugXYSurface - hugXYRhumb;
    angleError = (sonarRight - hugTargetDistance) * 20.0;  // positive values to right 
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing + angleError; 
  } else {
    hugTargetDistance = hugXYRhumb - hugXYSurface;
    angleError = (sonarLeft - hugTargetDistance) * 20.0;  // positive values to right 
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing - angleError; 
  }
  steerHeading();
  
//  
//  double distanceError;
//  distanceError = isRightSonar ? (hugSonarDistance - sonarRight) : (sonarLeft - hugSonarDistance);
//  double angleCorrection = distanceError * 20.0D;
//  if (angleCorrection > 20) angleCorrection = 20.0D;
//
//  double aDiff = hugBearing - angleCorrection - currentMapHeading;
//  if (aDiff > 180.0) aDiff -= 360.0;
//  else if (aDiff < -180.0) aDiff += 360.0;
//  double d = (aDiff > 0.0) ? 1.0 : -1.0;
//
//  double speedAdjustment = (wheelSpeedFps / (RAD_TURN * 3.0)) * 0.64 * d;
//
//  // Reduce adjustment proportionally if less than X degrees.
//  if (abs(aDiff) < 2.0) {
//    speedAdjustment = (abs(aDiff) / 2.0) * speedAdjustment;
//  }
//
//  tp6FpsRight = tp6Fps - speedAdjustment;
//  tp6FpsLeft = tp6Fps + speedAdjustment;
//  //  addLog(
//  //    (long) timeMicroseconds,
//  //    (short) (currentMapHeading * 100.0),
//  //    (short) (speedAdjustment * 100.0),
//  //    (short) (sonarLeft * 100.0),
//  //    (short) (aDiff * 100.0),
//  //    (short) (currentMapLoc.x * 100.0),
//  //    (short) (currentMapLoc.y * 100.0)
//  //  );
//  //
}



/************************************************************************
 *  stopFix() Stop  and fix location on target X or Y.
 *  Return true if stabile or escape message received.
 ************************************************************************/
boolean stopFix() {
  double speedAdjustment = 0.0;
  routeFps = 0.0;

  // Adjust differential wheel speeds to point in correct direction
  stopADiff = rangeAngle(routeTargetBearing - currentMapHeading);
  speedAdjustment = stopADiff * .1;
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;

  // Adjust routeFps to set position.
  if (axisC == 'X') {
    stopDist = currentMapLoc.x - routeTargetLoc.x;
  }
  else {
    stopDist = currentMapLoc.y - routeTargetLoc.y;
  }
  if (!isRouteTargetIncreasing) stopDist *= -1.0;  // P
  double fps = tp6LpfCos;                      // D
  routeFps = (stopDist * 4.0) - (fps * 2.3);    // P+D

  // Check to see if we are stabile.
  if (isRouteWait) {
    if (isEsReceived) {
      isEsReceived = false;
      return true;
    }
    return false;
  }
  else {
    if ((abs(gaPitch) < 1.0) &&
        (abs(tp6LpfCos) < 0.1) &&
        (abs(stopADiff) < 1.0) &&
        (abs(wheelSpeedFps) < 0.3) &&
        (abs(stopDist) < 0.25))  return true;
    return false;
  }
}



/************************************************************************
 *  discombobulate()
 ************************************************************************/
void discombobulate() {
  double aDiff = routeTargetBearing - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  double speedAdjustment = aDiff * (S_LIM / A_LIM);
  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
  tp6FpsLeft = tp6Fps + speedAdjustment;
  tp6FpsRight = tp6Fps - speedAdjustment;
}



/************************************************************************
 *  turnRadius() Turn with a given radius.
 *  reFactor += .4;

 ************************************************************************/
void turnRadius() {
  double speedAdjustment = 0.0;
  double aDiff  = 0.0;
  double d;
  double radiusError = 0.0D;
  double  headingError = 0.0D;

  if (isTurnDegrees) {
    d = routeIsRightTurn ? 1.0 : -1.0;
  }
  else { // Turn toward the routeTarget
    aDiff = routeTargetBearing - currentMapHeading;
    if (aDiff > 180.0) aDiff -= 360.0;
    else if (aDiff < -180.0) aDiff += 360.0;
    d = (aDiff > 0.0) ? 1.0 : -1.0;
  }
  double rsa = (wheelSpeedFps / routeRadius) * 0.64 * d;

  double xDist = (pivotLoc.x - currentMapLoc.x);
  double yDist = (pivotLoc.y - currentMapLoc.y);
  radiusError = sqrt((xDist * xDist) + (yDist * yDist)) - routeRadius;
  int turnTicks = tickPosition - startTurnTick;
  headingError = (turnTicks * degreesPerTick) - currentMapHeading;
  double errorAdjustment = radiusError * reFactor * d;
  speedAdjustment = rsa + errorAdjustment;

  //  static int loopR = 0;
  //  if ((++loopR % 10) == 0) {
  //   addLog(
  //    (long) (errorAdjustment * 100.0),
  //    (short) (currentMapLoc.x * 100.0),
  //    (short) (currentMapLoc.y * 100.0),
  //    (short) (radiusError * 1000.0),
  //    (short) (rsa * 100.0),
  //    (short) (speedAdjustment * 100.0),
  //    (short) routeStepPtr
  //  );
  //  }

  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



///************************************************************************
// *  orient() Orient to magnetic direction and hold position.
// ************************************************************************/
//void orient() {
//  double speedAdjustment = 0.0;
//
//  double aDiff = routeTargetMagHeading - currentMapHeading;
//  if (aDiff > 180.0) aDiff -= 360.0;
//  else if (aDiff < -180.0) aDiff += 360.0;
//
//  speedAdjustment = aDiff * (S_LIM / A_LIM);
//  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
//
//  tp6FpsRight = tp6Fps - speedAdjustment;
//  tp6FpsLeft = tp6Fps + speedAdjustment;
//
//  routeFps = ((float) (standPos - tickPosition)) * 0.0005;
//}



/************************************************************************
 *  adjustPosition() Allow position and bearing to be adjusted
 *                   from the joystick.  If M, then only allow position.
 ************************************************************************/
void adjustPosition() {
  static boolean isOldOnGround = false;
  float speedAdjustment = 0.0;
  routeFps = 0.0;
  double aDiff;

  // Adjust the position
  float joyY = (abs(pcY) > abs(hcY)) ? pcY : hcY;
  if (abs(joyY) > 0.05) {
    fixPosition = fixPosition + ((int) (joyY * 10.0));
  }
  int dist = coTickPosition - fixPosition; // P
  double fps = tp6LpfCos;                  // D
  routeFps = ((double) - (dist * .004)) - (fps * 2.0);

  // Adjust the heading.
  if (isMagAdjust) {
    //    speedAdjustment = -gridHeading * (S_LIM / A_LIM);
    //    speedAdjustment = -gmHeading * (S_LIM / A_LIM);
    speedAdjustment = -tmHeading * (S_LIM / A_LIM);
    speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
  } else {
    float joyX = (abs(pcX) > abs(hcX)) ? pcX : hcX;
    if (abs(joyX) > 0.05) {
      fixHeading = rangeAngle(fixHeading + (joyX * .2));
    }
    aDiff = rangeAngle(fixHeading - tickHeading);
    speedAdjustment = aDiff * .2;
  }
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  pirouette()   Routine for going over ~8" jumps; stairs or ramp.
 ************************************************************************/
boolean pirouette() {
  double speedAdjustment;
  if (routeIsRightTurn) {
    if (currentMapCumHeading > turnTargetCumHeading) return true;
    speedAdjustment = pirouetteFps;
  } else {
    if (currentMapCumHeading < turnTargetCumHeading) return true;
    speedAdjustment = -pirouetteFps;
  }
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
  return false;
}

/************************************************************************
 *  leastSquares()
 ************************************************************************/
void leastSquares() {
  if (lsPtr < 2) return;
  float df = (float) (lsPtr - 1);
  float sumX = 0.0F;
  float sumYL = 0.0F;
  float sumYS = 0.0F;
  float sumSqX = 0.0F;
  float sumSqYL = 0.0F;
  float sumSqYS = 0.0F;
  float sumXYL = 0.0F;
  float sumXYS = 0.0F;
  float meanX, meanYL, meanYS;
  float diffX, diffYL, diffYS, sigmaX, sigmaYL, sigmaYS;
  float rL, rS, slopeL, slopeS, interceptL, interceptS;
  double angleCorrection;

  for (int i = 0; i < lsPtr; i++) sumX += lsXArray[i];
  for (int i = 0; i < lsPtr; i++) sumYL += lsYLArray[i];
  for (int i = 0; i < lsPtr; i++) sumYS += lsSArray[i];
  meanX = sumX / ((float) lsPtr);
  meanYL = sumYL / ((float) lsPtr);
  meanYS = sumYS / ((float) lsPtr);
  for (int i = 0; i < lsPtr; i++) {
    diffX = meanX - lsXArray[i];
    sumSqX += diffX * diffX;
    diffYL = meanYL - lsYLArray[i];
    sumSqYL += diffYL * diffYL;
    diffYS = meanYS - lsSArray[i];
    sumSqYS += diffYS * diffYS;
    sumXYL += (diffX * diffYL);
    sumXYS += (diffX * diffYS);
  }
  sigmaX = sqrt(sumSqX / df);
  sigmaYL = sqrt(sumSqYL / df);
  sigmaYS = sqrt(sumSqYS / df);
  rL = (sumXYL / (sigmaYL * sigmaX)) / df;
  slopeL = rL * (sigmaYL / sigmaX);
  double bearingL = asin(slopeL) * RAD_TO_DEG;
  interceptL = meanYL - (slopeL * meanX);
  rS = (sumXYS / (sigmaYS * sigmaX)) / df;
  slopeS = rS * (sigmaYS / sigmaX);
  double bearingS = asin(slopeS) * RAD_TO_DEG;
  interceptS = meanYS - (slopeS * meanX);

  if (isHug) { // Following V command. Use last sonar for xy value;
    if (!isRightSonar) {
      if (isGXAxis) {
        if (isRouteTargetIncreasing) {
          sendBMsg(SEND_MESSAGE, "Left, !XAxis, increasing: East");
          currentMapLoc.y = lsXYSurface - sonarLeft;
          angleCorrection = bearingL + bearingS;
          setHeading(currentMapHeading + angleCorrection);
        } else {
          sendBMsg(SEND_MESSAGE, "Left, XAxis, !increasing: West");
          currentMapLoc.y = lsXYSurface + sonarLeft;
          angleCorrection = bearingL - bearingS;
          setHeading(currentMapHeading + angleCorrection);
        }
      } else {
        if (isRouteTargetIncreasing) {
          // Correct from some testing.
          sendBMsg(SEND_MESSAGE, "Left, !XAxis, increasing: North");
          currentMapLoc.x = lsXYSurface + sonarLeft;
          angleCorrection = bearingL - bearingS;
          setHeading(currentMapHeading - angleCorrection);
        } else {
          sendBMsg(SEND_MESSAGE, "Left, !XAxis, !increasing: South");
          currentMapLoc.x = lsXYSurface - sonarLeft;
          angleCorrection = bearingL + bearingS;
          setHeading(currentMapHeading - angleCorrection);
        }
      }
    } else {
      // Right sonar.
      if (isGXAxis) {
        if (isRouteTargetIncreasing) {
           sendBMsg(SEND_MESSAGE, "Right, !XAxis, increasing: East");
          currentMapLoc.y = lsXYSurface + sonarRight;
          angleCorrection = bearingL - bearingS;
          setHeading(currentMapHeading + angleCorrection);
        } else {
          sendBMsg(SEND_MESSAGE, "Right, XAxis, !increasing: West");
          currentMapLoc.y = lsXYSurface - sonarRight;
          angleCorrection = bearingL + bearingS;
          setHeading(currentMapHeading + angleCorrection);
        }
      } else {
        if (isRouteTargetIncreasing) {
          sendBMsg(SEND_MESSAGE, "Right, !XAxis, increasing: North");
          currentMapLoc.x = lsXYSurface - sonarRight;
          angleCorrection = bearingL + bearingS;
          setHeading(currentMapHeading - angleCorrection);
        } else {
          sendBMsg(SEND_MESSAGE, "Right, !XAxis, !increasing: South");
          currentMapLoc.x = lsXYSurface + sonarRight;
          angleCorrection = bearingL - bearingS;
          setHeading(currentMapHeading - angleCorrection);
        }
      }
    }
  } else { // Following G command.
    // This situation only used for testing and callibration.  It only tests
    // traveling East with the left sonar.
    currentMapLoc.y = lsXYSurface - sonarLeft;
    angleCorrection = bearingL + bearingS;
    setHeading(currentMapHeading + angleCorrection);
    //    sprintf(message, "Dist. correction:%2.2f",  (lsDistanceMeasured - lsDistanceExpected));
    //    sendBMsg(SEND_MESSAGE, message);
  }
  //  sprintf(message, "%5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f",
  //          sigmaX, sigmaYL, sigmaYS, rL, bearingL, interceptL, rS, bearingS, interceptS);
  //  sendBMsg(SEND_MESSAGE, message);
  sprintf(message, "bearingL: %5.2f\tbearingS: %5.2f", bearingL, bearingS);
  sendBMsg(SEND_MESSAGE, message);
  sprintf(message, "Angle correction:%2.2f",  angleCorrection);
  sendBMsg(SEND_MESSAGE, message);
}



/************************************************************************
 *  setTargetBearing() Set the new XY position given the distance and
 *                orientation traveled since the last call.
 ************************************************************************/
void setTargetBearing() {
  double x =  routeTargetLoc.x - currentMapLoc.x;
  double y = routeTargetLoc.y - currentMapLoc.y;
  routeTargetBearing = atan2(x, y) * RAD_TO_DEG;
}


boolean isReachedTrigger(double trigger) {
  double gValue = isGXAxis ? currentMapLoc.x : currentMapLoc.y;
  targetDistance = trigger - gValue;
  if (!isRouteTargetIncreasing) targetDistance = -targetDistance;
  if (targetDistance <= 0.0) return (true);
  return (false);
}


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
