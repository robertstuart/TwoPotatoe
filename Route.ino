
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

void routeLog() {
  addLog(
    (long) (currentMapLoc.x * 100.0),
    (short) (currentMapLoc.y * 100.0),
    (short) (routeTargetLoc.x * 100.0),
    (short) (routeTargetLoc.y * 100.0),
    (short) (phantomTargetLoc.x * 100.0),
    (short) (phantomTargetLoc.y * 100.0),
    (short) (phantomTargetBearing * 100.0)
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
    case 'N':
    case 'Z':
    case 'E':
    case 'W':
    case 'H':
      isNewRouteStep = true;
      break;

    case 'A':  // Adjust position and bearing
      if (isEndStand()) {
        isNewRouteStep = true;
        setHeading(0.0D);
        setLoc(0.0D, 0.0D);
      }
      else adjustPosition();
      break;

    case 'D':  // Discombobulator, going south.
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      if (routeTargetXYDistance >= 0.0) isNewRouteStep = true;
      discombobulate();
      break;

    case 'K':  // Discombobulator, going north.
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      if (routeTargetXYDistance <= 0.0) isNewRouteStep = true;
      discombobulate();
      break;

    case 'P':
      if (jump()) isNewRouteStep = true;
      break;

    case 'O': // Orient
      if (!isRouteWait) {
        if (abs(currentMapHeading - routeTargetBearing) < 1) isNewRouteStep = true;
      }
      if (isEndStand()) isNewRouteStep = true;
      orient();
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

    case 'x': // CX - Moving toward X axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x;
      routeCoDistance = routeCoDistanceXY - currentMapLoc.x;
      if (isRouteTargetIncreasing) {
        if (routeCoDistance <= 0.0) {
          isNewRouteStep = true;
          doChartedObject();
        }
      } else {
        if (routeCoDistance >= 0.0) {
          isNewRouteStep = true;
          doChartedObject();
        }
      }
      steerHeading();
      break;

    case 'X': // GX - Move toward X axis
      if (abs(routeFps) < 0.1) isNewRouteStep = true;
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x;
      if (isRouteTargetIncreasing) {
        if (routeTargetXYDistance <= 0.0) isNewRouteStep = true;
      }
      else {
        if (routeTargetXYDistance >= 0.0) isNewRouteStep = true;
      }
      steerHeading();
      break;
    case 'y': // CY - Moving toward Y axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      routeCoDistance = routeCoDistanceXY - currentMapLoc.y;
      //      sprintf(message, "%5.2f", routeCoDistance ); isNewMessage = true;
      if (isRouteTargetIncreasing) {
        if (routeCoDistance <= 0.0) {
          isNewRouteStep = true;
          doChartedObject();
        }
      }
      else {
        if (routeCoDistance >= 0.0) {
          isNewRouteStep = true;
          doChartedObject();
        }
      }
      steerHeading();
      break;

    case 'Y': // GY - Move toward Y axis
      if (abs(routeFps) < 0.1) isNewRouteStep = true;
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      if (isRouteTargetIncreasing) {
        if (routeTargetXYDistance <= 0.0) isNewRouteStep = true;
      }
      else {
        if (routeTargetXYDistance >= 0.0) isNewRouteStep = true;
      }
      steerHeading();
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
        Serial.print(routeCoDistanceXY); Serial.print("   ");
        if ((gridOffset < -180.0D) || (gridOffset > 180.0)) return false;
      }
      fixPosition = tickPosition;
      fixHeading = tickHeading;
      routeTargetLoc.y = 100.0; // For following command.
      routeTargetLoc.x = 0.0;
      setPhantomTarget();
      break;

    case 'C': // Charted object
      char1 = stepString.charAt(0);
      if (char1 == 'X') routeCurrentAction = 'x';
      else if (char1 == 'Y') routeCurrentAction = 'y';
      else return false;
      stepString = stepString.substring(1);

      routeCoDistanceXY = readNum();
      Serial.print(routeCoDistanceXY); Serial.print("   ");
      if (routeCoDistanceXY == STEP_ERROR) return false;

      routeSonarMin = readNum();
      Serial.print(routeSonarMin); Serial.print("   ");
      if (routeSonarMin == STEP_ERROR) return false;

      routeSonarMax = readNum();
      Serial.print(routeSonarMax); Serial.print("   ");
      if (routeSonarMax == STEP_ERROR) return false;

      routeSonarDist = readNum();
      Serial.print(routeSonarDist); Serial.print("   ");
      if (routeSonarDist == STEP_ERROR) return false;

      while (true) {
        char1 = readChar();
        Serial.print(char1); Serial.print(" ");
        if (char1 == 'i') { // Save for later Or-i-entation  fix?
          isSaveOrientation = true;
        }
        else if (char1 == 's') { // Save for later Po-s-ition fix?
          isSavePosition = true;
        }
        else if (char1 == 'o') { // Fix the map orientation?
          isFixOrientation = true;
        }
        else break;
      }
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
      break;

    case 'G':
      char1 = stepString.charAt(0);
      if (char1 == 'X') routeCurrentAction = 'X';
      else if (char1 == 'Y') routeCurrentAction = 'Y';
      else return false;
      stepString = stepString.substring(1);

      routeTargetLoc = readLoc();
      Serial.print(routeTargetLoc.x); Serial.print("  "); Serial.print(routeTargetLoc.y); Serial.print("   ");
      if (routeTargetLoc.y == STEP_ERROR) return false;
      if (routeCurrentAction == 'X') {
        routeTargetXY = routeTargetLoc.x;
        if (currentMapLoc.x < routeTargetLoc.x) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      else {
        routeTargetXY = routeTargetLoc.y;
        if (currentMapLoc.y < routeTargetLoc.y) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      setPhantomTarget();

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

    case 'N':
      stripWhite();
      routeTitle = stepString;
      break;

    case 'O':  // Orient
      char1 = stepString.charAt(0);
      if (char1 == 'W') {
        isRouteWait = true;
        stepString = stepString.substring(1);
      }
      else {
        isRouteWait = false;
      }
      routeTargetMagHeading = readNum();
      Serial.print(routeTargetMagHeading); Serial.print("  "); 
      if (routeTargetMagHeading == STEP_ERROR) return false;
      standPos = tickPosition;
      break;

    case 'P':
      isStepFall = isOldStepFall = false;
      routeFps = 1.1;
      jumpTarget = currentMapLoc.y + 2.3D; // Eventually stop if no jump.
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
        if (phantomTargetLoc.x < routeTargetLoc.x) isRouteTargetIncreasing = true;
        else isRouteTargetIncreasing = false;
      }
      else {
        routeTargetXY = routeTargetLoc.y;
        if (phantomTargetLoc.y < routeTargetLoc.y) isRouteTargetIncreasing = true;
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
      sprintf(message, "pivotLoc.x: %5.2f     pivotLoc.y: %5.2f", pivotLoc.x, pivotLoc.y);
      sendBMsg(SEND_MESSAGE, message);
      Serial.println(); 
      break;

    case 'W':  // dup of G
      char1 = stepString.charAt(0);
      if (char1 == 'X') axisC = 'X';
      else if (char1 == 'Y') axisC = 'Y';
      else return false;
      stepString = stepString.substring(1);

      routeTargetLoc = readLoc();
      Serial.print(routeTargetLoc.x); Serial.print("  "); Serial.print(routeTargetLoc.y); Serial.print("   ");
      if (routeTargetLoc.y == STEP_ERROR) return false;
      if (axisC == 'X') {
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
  double angleCorrection = 0.0;
  double previousPosXY = 0.0;
  double lateralError = 0.0;
  double dist = 0.0;
  boolean isHeadingNorth = true;
  boolean isHeadingEast = true;
  boolean isGoodReading = true;

  //  routeLog(); // Before

  if ((sonarRight < routeSonarMin) || (sonarRight > routeSonarMax)) isGoodReading = false;

  // Adjust the XY location
  if (routeCurrentAction == 'x') {
    if (currentMapHeading < 0.0) isHeadingEast = false;
    if (isHeadingEast) {
      lateralError = sonarRight - routeSonarDist;
    }
    else {
      lateralError = routeSonarDist - sonarRight;
    }
    if (isGoodReading) currentMapLoc.y += lateralError;
  }
  else {
    if ((currentMapHeading < -90.0) || (currentMapHeading > 90.0)) isHeadingNorth = false;
    if (isHeadingNorth) {
      lateralError = routeSonarDist - sonarRight;
    }
    else {
      lateralError = sonarRight - routeSonarDist;
    }
    if (isGoodReading) currentMapLoc.x += lateralError;
  }


  if (isSaveOrientation) { // 'i'
    if (isGoodReading) {
      savedOrientationLoc = currentMapLoc;
    }
    else {
      savedOrientationLoc.x = INVALID_VAL;
      savedOrientationLoc.y = INVALID_VAL;
    }
    isSaveOrientation = false;
  }
  if (isSavePosition) { // 's'
    if (isGoodReading) {
      savedPositionLoc = currentMapLoc;
    }
    else {
      savedPositionLoc.x = INVALID_VAL;
      savedPositionLoc.y = INVALID_VAL;
    }
    isSavePosition = false;
  }

  // Adjust the heading
  if (isGoodReading && (savedOrientationLoc.x != INVALID_VAL)) {
    if (isFixOrientation) { // 'o'
      if (routeCurrentAction == 'x') {
        if (isHeadingEast) {
          dist = savedOrientationLoc.x - currentMapLoc.x;
        }
        else {
          dist = currentMapLoc.x - savedOrientationLoc.x;
        }

      }
      else {
        if (isHeadingNorth) {
          dist = currentMapLoc.y - savedOrientationLoc.y;
        }
        else {
          dist = savedOrientationLoc.y - currentMapLoc.y;
        }
      }
      angleCorrection = asin(lateralError / dist) * RAD_TO_DEG;
      gyroCumHeading += angleCorrection;
    }
  }
  //  routeLog(); // After

  //  sprintf(message, "%5.2f %5.2f", sonarRight, routeSonarDist);
  //  sprintf(message, "%5.2f %5.2f %5.2f", sonarRight, routeSonarDist, angleCorrection);
  //  sprintf(message, "%5.2f %5.2f %5.2f %5.2f", sonarRight, routeSonarDist, dist, angleCorrection);
  //  isNewMessage = true;
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
  //  static double oldTb = 0.0;
  //  double tb;  // Target bearing
  //
  //  // No turning when closer than X.X feet.
  //  if (abs(routeTargetXYDistance) < END_STEER)  tb = oldTb;
  //  else oldTb = tb = routeTargetBearing;
  //  if (routeFps < 0.0) tb += 180.0;
  //
  //  aDiff = tb - currentMapHeading;
  //
  double aDiff = phantomTargetBearing - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  double d = (aDiff > 0.0) ? 1.0 : -1.0;

  double speedAdjustment = (wheelSpeedFps / RAD_TURN) * 0.64 * d;

  // Reduce adjustment proportionally if less than X degrees.
  if (abs(aDiff) < 2.0) {
    speedAdjustment = (abs(aDiff) / 2.0) * speedAdjustment;
  }

  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  stopFix() Stop  and fix location on target X or Y.
 *  Return true if stabile or escape message received.
 ************************************************************************/
boolean stopFix() {
  double speedAdjustment = 0.0;
  routeFps = 0.0;

  // Adjust differential wheel speeds to point in correct direction
  stopADiff = rangeAngle(phantomTargetBearing - currentMapHeading);
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
 ************************************************************************/
void turnRadius() {
  double speedAdjustment = 0.0;
  double aDiff  = 0.0;
  double d;
  if (isTurnDegrees) {
    d = routeIsRightTurn ? 1.0 : -1.0;
  }
  else { // Turn toward the routeTarget
    aDiff = routeTargetBearing - magHeading;
    if (aDiff > 180.0) aDiff -= 360.0;
    else if (aDiff < -180.0) aDiff += 360.0;
    d = (aDiff > 0.0) ? 1.0 : -1.0;
  }
//  double xDist = currentMapLoc.x - pivotLoc.x;
//  double yDist = currentMapLoc.y - pivotLoc.y;
//  double dDiff = sqrt((xDist * xDist) + (yDist * yDist)) - routeRadius;
//  double targetAngle = (atan2(xDist, yDist) * RAD_TO_DEG) + 90.0;
//  targetAngle = rangeAngle(targetAngle);
//  aDiff = rangeAngle(targetAngle - currentMapHeading);
  speedAdjustment = (wheelSpeedFps / routeRadius) * 0.64 * d;
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  orient() Orient to magnetic direction and hold position.
 ************************************************************************/
void orient() {
  double speedAdjustment = 0.0;

  double aDiff = routeTargetMagHeading - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;

  speedAdjustment = aDiff * (S_LIM / A_LIM);
  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);

  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;

  routeFps = ((float) (standPos - tickPosition)) * 0.0005;
}



/************************************************************************
 *  adjustPosition() Allow position and bearing to be adjusted
 *                   from the joystick.
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
    speedAdjustment = gridHeading * (S_LIM / A_LIM);
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
 *  jump()   Routine for going over ~8" jumps; stairs or ramp.
 *             1. Set currenValSet.v to 5.0. Go to target at 2.2fps
 *             2. At 1st !isOnGround, mark Y distance
 *             3. At ~10" from drop, go to 0.0 fps.
 *             4. At < 0.2 fps, go to next step (S?).
 ************************************************************************/
boolean jump() {
  boolean ret = false;
  if (isStepFall) {  // Fall has happened.
    if (!isOldStepFall) { // The first time?
      jumpFallXY = currentMapLoc.y;
      jumpTarget = currentMapLoc.y + 0.7;  // ----------CRITICAL! --------------
    }
  }
  //  if (currentMapLoc.y > (jumpTarget - 0.2)) {
  //    (*currentValSet).v = 5.0;  // increase speed correction.
  //  }
  if (currentMapLoc.y > jumpTarget) {
    ret = true;
  }
  steerHeading();
  isOldStepFall = isStepFall;
  return ret;
}



/************************************************************************
 *  setTargetBearing() Set the new XY position given the distance and
 *                orientation traveled since the last call.
 ************************************************************************/
void setTargetBearing() {
  // Set the bearing to the target
  double x =  routeTargetLoc.x - currentMapLoc.x;
  double y = routeTargetLoc.y - currentMapLoc.y;
  routeTargetBearing = atan2(x, y) * RAD_TO_DEG;

  // Set the bearing to the phantomTarget
  x = phantomTargetLoc.x - currentMapLoc.x;
  y = phantomTargetLoc.y - currentMapLoc.y;
  phantomTargetBearing = atan2(x, y) * RAD_TO_DEG;
}


#define PHANTOM_DIST 15.0 // feet beyond actual target
/************************************************************************
 *  setPhantomTarget() This sets the phantom target which is X.X feet
 *                     beyond the actual target.  This is to prevent
 *                     rapid turning as tp approaches target.
 ************************************************************************/
void setPhantomTarget() {
  double distX =  routeTargetLoc.x - currentMapLoc.x;
  double distY = routeTargetLoc.y - currentMapLoc.y;
  double rtb = atan2(distY, distX);
  double pDist = sqrt((distX * distX) + (distY * distY)) + PHANTOM_DIST;
  phantomTargetLoc.x = (cos(rtb) * pDist) + currentMapLoc.x;
  phantomTargetLoc.y = (sin(rtb) * pDist) + currentMapLoc.y;
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
