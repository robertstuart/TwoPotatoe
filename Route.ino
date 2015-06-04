
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

void routeLog() {
  addLog(
    (long) timeMicroseconds,
    (short) (currentMapLoc.x * 100.0),
    (short) (currentMapLoc.y * 100.0),
    (short) (currentMapHeading * 100.0),
    (short) (routeTargetLoc.x * 100.0),
    (short) (routeTargetLoc.y * 100.0),
    (short) (routeTargetBearing * 100.0)
  );
}

/************************************************************************
 *  Route() called every loop (400/sec)
 *          1. Check if target reached.
 *          2. Set currentLoc.
 *          3. Adjust steering.
 ************************************************************************/
void route() {
  boolean isNewRouteStep = false;
  setTargetBearing();

  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'N':
    case 'M':
    case 'Z':
    case 'W':
      isNewRouteStep = true;
      break;
    case 'S': // Stand
      if (digitalRead(GN_SW_PIN) == LOW) isNewRouteStep = true;
      if (digitalRead(YE_SW_PIN) == LOW) isNewRouteStep = true;
      if (isEsReceived) {
        isNewRouteStep = true;
        isEsReceived = false;
      }
      if (routeWaitTime < timeMilliseconds) isNewRouteStep = true;
      orient();
      break;
    case 'x': // CX - Moving toward X axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x;
      routeCoDistance = routeCoDistanceXY - currentMapLoc.x;
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
    case 'X': // GX - Move toward X axis
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
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y;
      if (isRouteTargetIncreasing) {
        if (routeTargetXYDistance <= 0.0) isNewRouteStep = true;
      }
      else {
        if (routeTargetXYDistance >= 0.0) isNewRouteStep = true;
      }
      steerHeading();
      break;
    case 'T': // Turn
      if (abs(routeTargetBearing - currentMapHeading) < 5.0)  isNewRouteStep = true;
      turnRadius();
      break;
    default:
      isRouteInProgress = false;
      sprintf(message, "Illegal step in route()"); isNewMessage = true;
      break;
  } // end switch()

  // Move to new action if current action is done.
  if (isNewRouteStep) { // Move to next action in list.
    interpretRouteLine(getNextStepString());
  }
}


/************************************************************************
 *  interpretRouteLine()
 ************************************************************************/
boolean interpretRouteLine(String ss) {
  double retDbl;
  int retInt;
  char charX;
  char axisC;
  
  stepString = ss;
  isSaveOrientation = isSavePosition = isFixOrientation = false;
//  stepString = garageLoop[routeStepPtr++];
//  stepString = getNextStepString();
  originalStepStringPtr = 0;
  Serial.print(stepString);  Serial.print(":   ");
  routeCurrentAction = stepString.charAt(0);
  stepString = stepString.substring(1);
  originalStepStringPtr++;

  switch (routeCurrentAction) {
    case 'N':
      stripWhite();
      routeTitle = stepString;
      break;

    case 'M':
      mapOrientation = readNum();
      Serial.print(mapOrientation);
      if (mapOrientation == STEP_ERROR) return false;
      break;

    case 'Z':
      mapOrientation = readNum();
      Serial.print(mapOrientation);
      if (mapOrientation == STEP_ERROR) return false;
      resetNavigation(0.0);
      break;

    case 'S':  // Stand
      routeTargetBearing = readNum();
      Serial.print(routeTargetBearing); Serial.print("  ");
      if (routeTargetBearing == STEP_ERROR) return false;
      routeTargetLoc.x = sin(routeTargetBearing * DEG_TO_RAD) * 100.0;
      routeTargetLoc.y = cos(routeTargetBearing * DEG_TO_RAD) * 100.0;

      retDbl = readNum();
      if (retDbl == STEP_ERROR) return false;
      Serial.print(retDbl);
      routeWaitTime = timeMilliseconds + ((int) retDbl) * 1000;
      routeFps = 0.0;
      break;

    case 'G':
      charX = stepString.charAt(0);
      if (charX == 'X') routeCurrentAction = 'X';
      else if (charX == 'Y') routeCurrentAction = 'Y';
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

      routeFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;

      routeRadius = readNum();
      Serial.print(routeRadius);  Serial.print("   ");
      if (routeRadius < .5) routeRadius = 0.5;
      if (routeRadius == STEP_ERROR) routeRadius = 2.0;
      break;


    case 'W':  // dup of G
      charX = stepString.charAt(0);
      if (charX == 'X') axisC = 'X';
      else if (charX == 'Y') axisC = 'Y';
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

    case 'C':
      charX = stepString.charAt(0);
      if (charX == 'X') routeCurrentAction = 'x';
      else if (charX == 'Y') routeCurrentAction = 'y';
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
        charX = readChar();
        Serial.print(charX); Serial.print(" ");
        if (charX == 'i') { // Save for later Or-i-entation  fix?
          isSaveOrientation = true;
        }
        else if (charX == 's') { // Save for later Po-s-ition fix?
          isSavePosition = true;
        }
        else if (charX == 'o') { // Fix the map orientation?
          isFixOrientation = true;
        }
        else break;
      }
      break;

    case 'T':  // Turn
      routeTargetLoc = readLoc();
      Serial.print(routeTargetLoc.x); Serial.print("  "); Serial.print(routeTargetLoc.y); Serial.print("   ");
      if (routeTargetLoc.y == STEP_ERROR) return false;

      routeFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;

      routeRadius = readNum();
      Serial.print(routeRadius);  Serial.print("   ");
      if (routeRadius < .5) routeRadius = 0.5;
      if (routeRadius == STEP_ERROR) routeRadius = 2.0D ;

      setTargetBearing();
      aDiff = routeTargetBearing - currentMapHeading;
      if (aDiff > 180.0) aDiff -= 360.0;
      else if (aDiff < -180.0) aDiff += 360.0;
      routeIsRightTurn = aDiff > 0.0;
      routeStartTickTurn = tickPosition;
      break;

    case 'R':
      routeStepPtr = (int) readNum();
      Serial.print(retInt);
      if ((routeStepPtr == STEP_ERROR) || (routeStepPtr >= (routeStepPtr - 2))) return false;
      break;

    case 'F':
      isRouteInProgress = false;
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

  routeLog();
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
    if ((currentMapHeading < -90.0) && (currentMapHeading > 90.0)) isHeadingNorth = false;
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

  //  sprintf(message, "%5.2f %5.2f", sonarRight, routeSonarDist);
  //  sprintf(message, "%5.2f %5.2f %5.2f", sonarRight, routeSonarDist, angleCorrection);
  //  sprintf(message, "%5.2f %5.2f %5.2f %5.2f", sonarRight, routeSonarDist, dist, angleCorrection);
  //  isNewMessage = true;
}


/************************************************************************
 *  steerHeading() Find the correct heading to the target and adjust the
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 ************************************************************************/
void steerHeading() {
  static double oldTb = 0.0;
  double tb;

  if (abs(routeTargetXYDistance) < 5.0) { // No turning when closer than X.X feet.
    tb = oldTb;
  }
  else {
    oldTb = tb = routeTargetBearing;
  }
  if (routeFps < 0.0) tb += 180.0;
  aDiff = tb - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;

  speedAdjustment = aDiff * (S_LIM / A_LIM);
  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);

  tp6FpsLeft = tp6Fps + speedAdjustment;
  tp6FpsRight = tp6Fps - speedAdjustment;
}



/************************************************************************
 *  zeroMapHeading()
 ************************************************************************/
boolean zeroMapHeading() {
  boolean st = isRunReady;
  isRunReady = false;
  delay(200);
  readCompass();
  resetNavigation(magHeading);
  isRunReady = st;

  //  static unsigned int magTimeout = 0U;
  //  static double magSum = 0.0D;
  //  static int magCount = 0;
  //
  //  aDiff = routeMagTargetBearing - magHeading;
  //  if (aDiff > 180.0) aDiff -= 360.0;
  //  else if (aDiff < -180.0) aDiff += 360.0;
  //  speedAdjustment = aDiff * (S_LIM / A_LIM);
  //  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);
  //
  //  tp6FpsLeft = tp6Fps + speedAdjustment;
  //  tp6FpsRight = tp6Fps - speedAdjustment;
  //
  //  if (!isReachedMagHeading) {
  //    if (abs(aDiff) < 1.0) {
  //      isReachedMagHeading = true;
  //      magTimeout = timeMilliseconds + 2000;
  //      magSum = 0.0D;
  //      magCount = 0;
  //    }
  //  }
  //  else {
  //    magSum += magHeading;
  //    magCount++;
  //    if (timeMilliseconds > magTimeout) {
  //      double mh = magSum / ((double) magCount);
  //      resetNavigation(mh);
  //      return true;
  //    }
  //  }
  //  return false;
}

/************************************************************************
 *  turnRadius()
 ************************************************************************/
void turnRadius() {
  //  turnTickProgress = tickPosition - routeStartTickTurn;
  //  turnTargetBearing = (((double) turnTickProgress) * 0.01) / routeRadius;
  //  aDiff = turnTargetBearing - currentMapHeading;
  //  if (aDiff > 180.0) aDiff -= 360.0;
  //  else if (aDiff < -180.0) aDiff += 360.0;
  //  turnTrim = aDiff * 1.0;

  aDiff = routeTargetBearing - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;
  double d = (aDiff > 0.0) ? 1.0 : -1.0;

  speedAdjustment = (wheelSpeedFps / routeRadius) * 0.55 * d;

  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}

/************************************************************************
 *  orient()
 ************************************************************************/
void orient() {
  aDiff = routeTargetBearing - currentMapHeading;
  if (aDiff > 180.0) aDiff -= 360.0;
  else if (aDiff < -180.0) aDiff += 360.0;

  speedAdjustment = aDiff * (S_LIM / A_LIM);
  speedAdjustment = constrain(speedAdjustment, -S_LIM, S_LIM);

  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
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
