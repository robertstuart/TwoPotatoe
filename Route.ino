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
double avgTargetBearing = 0.0;
//double reFactor = 1.0;  // From tests at 3.5fps, 1.5 radius
//double reFactor = 2.0;  // From hous3
double reFactor = 0.0;  // From hous3
double startOrientation = 0.0;
struct loc startLoc;

void routeLog() {
  if (isRouteInProgress) {
    addLog(
      (long) (timeMilliseconds),
      (short) (currentMapLoc.x * 100.0),
      (short) (currentMapLoc.y * 100.0),
      (short) (gyroHeading * 100.0),
      (short) (sonarRight * 100.0),
      (short) (sonarLeft * 100.0),
      (short) routeStepPtr
    );
  }
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
  stickSpeed = 0.0;
  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'C':
    case 'E':
    case 'H':
    case 'L':
    case 'M':
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
        else setHeading(rangeAngle(tickHeading - fixHeading));
        //        setHeading(0.0D);
        currentMapLoc.x = 0.0D;
        currentMapLoc.y = 0.0D;
      }
      else adjustPosition();
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

    case 'K':  // Lock
      if (isEndStand()) {
        isNewRouteStep = true;
        setLockDrift();
        setHeading(rangeAngle(startOrientation));
        currentMapLoc = startLoc;
        run(true);
      }
      else {
        addLockHeading();
      }
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
        if (char2 == 'R') setSonar(SONAR_RIGHT);
        else if (char2 == 'L') setSonar(SONAR_LEFT);
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

    case 'M':
      char1 = stepString.charAt(0);
      if      (char1 == 'R') setSonar(SONAR_RIGHT);
      else if (char1 == 'L') setSonar(SONAR_LEFT);
      else if (char1 == 'B') setSonar(SONAR_BOTH);
      else if (char1 == 'N') setSonar(SONAR_NONE);
      else return false;
      break;

    case 'N':
      stripWhite();
      routeTitle = stepString;
      break;

    case 'P':  // Pirouette
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

    case 'S':  // Stop at a fixed point.
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
      if (routeRadius == STEP_ERROR) return false;
      
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

    case 'U': // Hug
      lsPtr = 0;
      isHug = true;
      hugDirection = stepString.charAt(0);
      if (hugDirection == 'N') {
        hugBearing = 0.0D;
        isGXAxis = false;
      }
      else if (hugDirection == 'E') {
        hugBearing = 90.0D;
        isGXAxis = true;
      }
      else if (hugDirection == 'S') {
        hugBearing = 180.0D;
        isGXAxis = false;
      }
      else if (hugDirection == 'W') {
        hugBearing = -90.0D;
        isGXAxis = true;
      }
      else return false;
      stepString = stepString.substring(1);
      char2 = stepString.charAt(0);
      if (char2 == 'R') setSonar(SONAR_RIGHT);
      else if (char2 == 'L') setSonar(SONAR_LEFT);
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
      Serial.println("Step error: Illegal command.");
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
  if (!isLifted) return true;
  return false;
}


/************************************************************************
 *  doChartedObject()
 ************************************************************************/
void doChartedObject() {
  struct chartedObject co = chartedObjects[coPtr];
  if (!isReachedTrigger(co.trigger)) return;

  double xyVal;
  int goodSonars[200];
  int nGood = 0;
  loc beforeLoc = currentMapLoc;

  // Compute median value of last  6 feet
  int nAll = (int) (133.0D / routeFps);      // number of measurement to collect
//  double minVal = 20.0D;
  if (co.isRightSonar) {
    for (int i = 0; i < nAll; i++) {
      int index = sonarRightArrayPtr - i - 1;
      if (index < 0) index += SONAR_ARRAY_SIZE; //% is not modulus!
      int val = sonarRightArray[index];
      if (val < 700) {
        goodSonars[nGood++] = val;
      }
    }
  } else {
    for (int i = 0; i < nAll; i++) {
      int index = sonarLeftArrayPtr - i - 1;
      if (index < 0) index += SONAR_ARRAY_SIZE; //% is not modulus!
      int val = sonarLeftArray[index];
      if (val < 700) {
        goodSonars[nGood++] = val;
      }
    }
  }
  coPtr++;
  if (nGood < 3) return;

  // Log the values
  for (int i = 0; i < nGood; i++) {
    addLog(goodSonars[i], 0,0,0,0,0,0);
  }

  // Find the median, Bubble Sort
  boolean swapped;
  while (true) {
    swapped = false;
    for (int i = 1; i < nGood; i++) {
      int a = goodSonars[i - 1];
      int b = goodSonars[i];
      if (a > b) {
        goodSonars[i - 1] = b;
        goodSonars[i] = a;
        swapped = true;
      }
    }
    if (!swapped) break;
  }
  float medianSonar = ((float) goodSonars[nGood / 2]) / 100.0;

  // Log the values
  for (int i = 0; i < nGood; i++) {
    addLog(0,goodSonars[i],0,0,0,0,0);
  }
  addLog(0,0,((short) (medianSonar * 100.0)),0,0,0,0);
  
//sprintf(message, "Sonar samples: %2d", n);
//sendBMsg(SEND_MESSAGE, message);

  if (!co.isRightSonar) {
    if (isGXAxis) {
      // Left sonar.
      if (isRouteTargetIncreasing) {
        currentMapLoc.y = co.surface - medianSonar;
      } else {
        currentMapLoc.y = co.surface + medianSonar;
      }
    } else {
      if (isRouteTargetIncreasing) {
        currentMapLoc.x = co.surface + medianSonar;
      } else {
        currentMapLoc.x = co.surface - medianSonar;
      }
    }
  } else {
    // Right sonar.
    if (isGXAxis) {
      if (isRouteTargetIncreasing) {
        currentMapLoc.y = co.surface + medianSonar;
      } else {
        currentMapLoc.y = co.surface - medianSonar;
      }
    } else {
      if (isRouteTargetIncreasing) {
        currentMapLoc.x = co.surface - medianSonar;
      } else {
        currentMapLoc.x = co.surface + medianSonar;
      }
    }
  }
  
  sprintf(message, "minVal: %5.2f\t  oldX: %5.2f\t oldY: %5.2f\t newX: %5.2f\t newY: %5.2f", 
  medianSonar, beforeLoc.x, beforeLoc.y, currentMapLoc.x, currentMapLoc.y);
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
  stickSpeed = controllerY * 5.0;
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  steerHug() Follow along a wall at the specified distance.
 *             Do not steer beyong 20 degrees from the specified angle.
 ************************************************************************/
void steerHug() {
  double hugTargetDistance, angleError;

  if ((hugDirection == 'N') && (sonarMode == SONAR_RIGHT)) {
    hugTargetDistance = hugXYSurface - hugXYRhumb;
    angleError = (sonarRight - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing + angleError; 
  } else if ((hugDirection == 'N') && (sonarMode == SONAR_LEFT)) {
    hugTargetDistance = hugXYRhumb - hugXYSurface;
    angleError = (sonarLeft - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing - angleError; 
  } else if ((hugDirection == 'S') && (sonarMode == SONAR_RIGHT)) {
    hugTargetDistance = hugXYRhumb - hugXYSurface;
    angleError = (sonarRight - hugTargetDistance) * 20.0; 
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing + angleError; 
  } else if ((hugDirection == 'S') && (sonarMode == SONAR_LEFT)) {
    hugTargetDistance = hugXYSurface - hugXYRhumb;
    angleError = (sonarLeft - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing - angleError; 
  } else if ((hugDirection == 'E') && (sonarMode == SONAR_RIGHT)) {
    hugTargetDistance = hugXYRhumb - hugXYSurface;
    angleError = (sonarRight - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing + angleError; 
  } else if ((hugDirection == 'E') && (sonarMode == SONAR_LEFT)) {
    hugTargetDistance = hugXYSurface - hugXYRhumb;
    angleError = (sonarLeft - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing - angleError; 
  } else if ((hugDirection == 'W') && (sonarMode == SONAR_RIGHT)) {
    hugTargetDistance = hugXYSurface - hugXYRhumb;
    angleError = (sonarRight - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing + angleError; 
  } else if ((hugDirection == 'W') && (sonarMode == SONAR_LEFT)) {
    hugTargetDistance = hugXYRhumb - hugXYSurface ;
    angleError = (sonarLeft - hugTargetDistance) * 20.0;
    angleError = constrain(angleError, -20.0, 20.0);
    routeTargetBearing = hugBearing - angleError; 
  }
  steerHeading();
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

  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



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
  if (abs(controllerY) > 0.05) {
    fixPosition = fixPosition + ((int) (controllerY * 10.0));
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
    if (abs(controllerX) > 0.05) {
      fixHeading = rangeAngle(fixHeading + (controllerX * .2));
    }
    aDiff = rangeAngle(fixHeading - tickHeading);
    speedAdjustment = aDiff * .2;
  }
  tp6FpsRight = tp6Fps - speedAdjustment;
  tp6FpsLeft = tp6Fps + speedAdjustment;
}



/************************************************************************
 *  pirouette()  
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

  if (isHug) { // Following U command. Use last sonar for xy value;
    if (sonarMode == SONAR_LEFT) {
      if (isGXAxis) {
        if (isRouteTargetIncreasing) {                                  // OK
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
        if (isRouteTargetIncreasing) {                                   // OK
          // Correct from some testing.
          sendBMsg(SEND_MESSAGE, "Left, !XAxis, increasing: North");
          currentMapLoc.x = lsXYSurface + sonarLeft;
          angleCorrection = bearingL - bearingS;
          setHeading(currentMapHeading - angleCorrection);
        } else {                                                         // OK
          sendBMsg(SEND_MESSAGE, "Left, !XAxis, !increasing: South");
          currentMapLoc.x = lsXYSurface - sonarLeft;
          angleCorrection = bearingL + bearingS;
          setHeading(currentMapHeading - angleCorrection);
        }
      }
    } else {
      // Right sonar.
      if (isGXAxis) {                                                  // OK
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
        if (isRouteTargetIncreasing) {                                  // OK
          sendBMsg(SEND_MESSAGE, "Right, !XAxis, increasing: North");
          currentMapLoc.x = lsXYSurface - sonarRight;
          angleCorrection = bearingL + bearingS;
          setHeading(currentMapHeading - angleCorrection);
        } else {                                                         // OK
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



/************************************************************************
 *  addTargetBearing() Average the target bearing over the last
 *                     1/4 second.
 ************************************************************************/
void addTargetBearing() {
  static const int AVG_SIZE = 100;
  static float avgArray[AVG_SIZE];
  static int avgPtr = 0;
  static float avgSum = 0.0;
  avgArray[avgPtr] = routeTargetBearing;
  avgSum += routeTargetBearing;
  avgSum -= avgArray[(avgPtr + AVG_SIZE - 1) % AVG_SIZE];
  avgTargetBearing = avgSum / ((float) AVG_SIZE);
  avgPtr = (avgPtr++ % AVG_SIZE);
}
#define LOCK_SIZE 400
double gLockSum = 0.0;
int gLockCount = 0;
/***********************************************************************.
 *  ???LockHeading()
 ***********************************************************************/
void addLockHeading() {
  static int gArray[LOCK_SIZE];
  static int gPtr = 0;
  int g, gMin, gMax;
  double gAve;

  gArray[gPtr] = lsm6.g.z;
  gPtr++;
  if (gPtr >= LOCK_SIZE) {
    int gSum = 0;
    gMax = gMin = gArray[0];
    for (int i = 0; i < LOCK_SIZE; i++) {
      g = gArray[i];
      if (g > gMax)  gMax = g;
      if (g < gMin) gMin = g;
      gSum += g;
    }
    gAve = ((double) gSum) / ((double) LOCK_SIZE);
    if ((gMax - gMin) < 90) {
      gLockSum += gAve;
      gLockCount++;
    }
Serial.print(gMax); Serial.print(tab);
Serial.print(gMin); Serial.print(tab);
Serial.print(gAve); Serial.println();
    gPtr = 0;
  }
}
void resetLockHeading() {
  timeDriftYaw = 0.0;
  gLockCount = 0;
  gLockSum = 0.0;
  lockStartTicks = tickPosition;
}
double setLockDrift() {
  timeDriftYaw = gLockSum / ((double) gLockCount);
Serial.print("Drift: "); Serial.println(timeDriftYaw);
}



/***********************************************************************.
 *  isReachedTrigger
 ***********************************************************************/
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
