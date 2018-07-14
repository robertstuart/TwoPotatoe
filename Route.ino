const float SEEK_FPS = 2.0;
const float TURN_FPS = 3.0;

// Barrel states
const int SEEK_BARREL = 0;
const int PLOT_BARREL = 1;
const int CIRCLE_BARREL = 2;
const int RECOVER_BARREL = 3;
int barrelOneState = SEEK_BARREL;

// Seek states
const int SEEK_SETTLE = 0;
const int SEEK_FWD = 1;
int seekState = SEEK_SETTLE;

// Plot states
const int PLOT_SETTLE = 0;
const int PLOT_LEFT = 1;
const int PLOT_RIGHT = 2;
int plotState = PLOT_SETTLE;

// Circle states
const int CIRCLE_ORIENT = 0;
const int CIRCLE_TURN = 2;
int circleState = CIRCLE_ORIENT;

const float STEP_ERROR = -42.42;
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

float targetBearing = 0.0;
float targetDistance = 0.0;
double degreesPerTick;
double turnStartBearing;
int startTurnTick;
double startOrientation = 0.0;
struct loc startLoc;
int jumpTicksEnd = 0;
float jumpCompFps = 0.0;

float coArray[50];
int coPtr = 0;
float coDist = 0.0;
float coHCorrection = 0.0;

boolean isHugRight = true;
float hugHeading = 0.0;
float hugDist = 0.0;
float hugWallDistance = 0.0;
float hugWallAngle = 0.0;
int hugStartTick = 0;
int hugEndTick = 0; 
const int HUG_ARRAY_SIZE = 100;
float hugSonarArray[HUG_ARRAY_SIZE] = {1.2, 3.5, 4.0, 5.0, 6.7, 8.9};
float hugDistArray[HUG_ARRAY_SIZE] =  {3.4, 3.7, 3.0, 2.7, 2.4, 2.5};
int hugPtr = 6;
float hugYEnd = 0.0;

void routeLog() {
  if (isRouteInProgress && isRunning) {
      addLog(
        (long) (timeMicroseconds),
        (short) (currentLoc.x * 100.0),
        (short) (currentLoc.y * 100.0),
        (short) (gcHeading * 100.0),
        (short) (sonarLeftKeep * 100.0),
        (short) (sonarFrontKeep * 100.0),
//        (short) (wFpsRight * 100.0),
//        (short) (routeFps * 100.0),
        (short) (routeStepPtr + (barrelOneState * 100) + (seekState * 1000) + (plotState * 10000))
      );
  }
}

/************************************************************************
 *  steerRoute() called every loop (208/sec).
 *          This is called as the last step in aTp7() for steering.
 *            1. Check if target reached.
 *            2. Set currentLoc.
 *            3. Adjust steering.
 ************************************************************************/
void steerRoute() {
  boolean isNewRouteStep = false;
  boolean ret;
  timeRun = timeMilliseconds - timeStart;
  targetWFpsRight = targetWFpsLeft = targetWFps;

  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'E':
    case 'L':
    case 'N':
    case 'D':
      isNewRouteStep = true;
      break;

    case 'B':
      if (barrels(false)) isNewRouteStep = true;
      break;
      
    case 'C':
      if (sonarRight > 0.1) {
        coArray[coPtr++] = sonarRight;
        sonarRight = 0.0;
        if (coPtr > 5) {
          coCorrect();
          isNewRouteStep = true;
        }
      } else {
        steerHeading();
      }
      break;
    case 'G':
      if (isTargetReached()) isNewRouteStep = true;
      else steerTarget();
      break;

    case 'H':
      if (tickPosition > hugEndTick) {
        hugCorrect();
        isNewRouteStep = true;
      } else {
        steerHug();
      }
      break;

    case 'J':
      if (tickPosition > jumpTicksEnd) isNewRouteStep = true;
      else jump(false);
      break;

    case 'K':  // Lock
      if (isStartReceived) {
        isNewRouteStep = true;
        setGyroDrift();
        setHeading(rangeAngle(startOrientation));
        currentLoc = startLoc;
        timeStart = timeMilliseconds;
        routeFps = 0.0;
        isRunReady = true;
      }
      else {
        routeFps = 0.0;
      }
      break;

    case 'P':
      if (pedestrian(false)) {
        isNewRouteStep = true;
      }
      break;

    case 'T': // Turn
      if (isTargetReached()) isNewRouteStep = true;
      else turn();
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
//    sendXMsg(SEND_MESSAGE, message);

  }
}



/************************************************************************
 *  interpretRouteLine()
 *      Called every time the end criterion for a route step is reached.
 *      Read the new route step and set the values.
 ************************************************************************/
boolean interpretRouteLine(String ss) {
  double pivotDir, dblVal, aDiff;
  float hugX, hugY, hugDist;
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
    case 'B': // Barrels
      barrelXCenter = readNum();
      Serial.print(barrelXCenter); Serial.print("   ");
      if (barrelXCenter == STEP_ERROR) return false;
      barrelYEnd = readNum();
      Serial.print(barrelYEnd); Serial.print("   ");
      if (barrelYEnd == STEP_ERROR) return false;
      barrels(true); // Reset
      break;

    case 'C': // Charted object
      targetBearing = readNum();
      Serial.print(targetBearing); Serial.print("   ");
      if (targetBearing == STEP_ERROR) return false;
      routeFps = routeScriptFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      coDist = readNum();
      Serial.print(coDist); Serial.print("   ");
      if (coDist == STEP_ERROR) return false;
      coHCorrection = readNum();
      Serial.print(coHCorrection); Serial.print("   ");
      if (coHCorrection == STEP_ERROR) return false;
      sonarRight = 0.0;
      coPtr = 0;
//      setSonar("lfR");
      break;

    case 'D':  // Decelerate
      decelFps = readNum();
      Serial.print(decelFps); Serial.print("   ");
      if (decelFps == STEP_ERROR) return false;
      isDecelActive = true;
      isDecelPhase = false;
      break;

    case 'F':  // Fini
      isRouteInProgress = false;
      break;

    case 'G':  // Go to the next waypoint
      targetLoc = readLoc();
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      routeFps = routeScriptFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      isDecelPhase = false;
      setTarget();
      break;

    case 'H':  // Hug a wall & correct heading and position.
      char1 = stepString.charAt(0);
      if (char1 == 'R') isHugRight = true;
      else if (char1 == 'L') isHugRight = false;
      else return false;
      stepString = stepString.substring(1);
      targetLoc = readLoc(); 
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      routeFps = routeScriptFps = readNum();
      Serial.print(routeFps);  Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      hugWallDistance = readNum(); 
      Serial.print(hugWallDistance); Serial.print("   ");
      if (hugWallDistance == STEP_ERROR) return false;
      hugWallAngle = readNum(); 
      Serial.print(hugWallAngle); Serial.print("   ");
      if (hugWallAngle == STEP_ERROR) return false;
      
      if (isHugRight) setSonar("Rfl");
      else setSonar("rfL");
      // Compute hugHeading & hugEndTick;
      hugStartLoc = currentLoc;
      hugX =  targetLoc.x  -currentLoc.x;
      hugY =  targetLoc.y - currentLoc.y;
      hugDist = sqrt((hugX * hugX) + (hugY * hugY));
      hugStartTick = tickPosition;
      hugEndTick = ((int) (TICKS_PER_FOOT * hugDist)) + hugStartTick; 
      hugHeading = atan2(hugX, hugY) * RAD_TO_DEG;
      hugPtr = 0;
      sonarRight = sonarRightKeep = 0.0;
      sonarLeft = sonarLeftKeep = 0.0;
      break;

    case 'J':  // Jump
      targetBearing = readNum();
      Serial.print(targetBearing); Serial.print("   ");
      if (targetBearing == STEP_ERROR) return false;
      routeFps = routeScriptFps = readNum();
      Serial.print(routeFps); Serial.print("   ");
      if (routeFps == STEP_ERROR) return false;
      jumpCompFps = readNum();
      Serial.print(jumpCompFps); Serial.print("   ");
      if (jumpCompFps == STEP_ERROR) return false;
      jump(true); // reset
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
      startGyroDrift();
      isStartReceived = false;
      break;

    case 'N':  // Name of route
      stripWhite();
      routeTitle = stepString;
      break;

    case 'P':  // Pedestrian
      pedestrian(true);
      break;

    case 'T':  // Turn at radius ending at waypoint.
      targetLoc = readLoc();
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      setTarget();
      aDiff = rangeAngle(targetBearing - gHeading);
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
      setTarget();
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
 *  steerTarget() Find the correct heading to the target and adjust the
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 ************************************************************************/
void steerTarget() {
  double speedAdjustment;

  if (targetDistance < 1.0) {
    speedAdjustment = 0.0;
  } else {
    double aDiff = rangeAngle(targetBearing - gHeading);
    double d = (aDiff > 0.0) ? 1.0 : -1.0;

    speedAdjustment = (wFps / RAD_TURN) * 0.64 * d;

    // Reduce adjustment proportionally if less than X degrees.
    if (abs(aDiff) < 5.0) {
      speedAdjustment = (abs(aDiff) / 5.0) * speedAdjustment;
    }

    // Reduce speed adjustment as speed increases
    if (tpFps > 11.0) speedAdjustment *= 0.3;
    else if (tpFps > 8.0) speedAdjustment *= 0.5;
    else if (tpFps > 5.0) speedAdjustment *= 0.7;
  }
  targetWFpsRight = targetWFps - speedAdjustment;
  targetWFpsLeft = targetWFps + speedAdjustment;
}


/***********************************************************************.
 *  steerHeading() Adjust the wheel speeds to turn toward the 
 *                 targetHeading. 
 ***********************************************************************/
void steerHeading() {
  double speedAdjustment;

  double aDiff = rangeAngle(targetBearing - gHeading);
  double d = (aDiff > 0.0) ? 1.0 : -1.0;

  speedAdjustment = (wFps / RAD_TURN) * 0.64 * d;

  // Reduce adjustment proportionally if less than X degrees.
  if (abs(aDiff) < 5.0) {
    speedAdjustment = (abs(aDiff) / 5.0) * speedAdjustment;
  }

  // Reduce speed adjustment as speed increases
  if (tpFps > 11.0) speedAdjustment *= 0.3;
  else if (tpFps > 8.0) speedAdjustment *= 0.5;
  else if (tpFps > 5.0) speedAdjustment *= 0.7;
    
  targetWFpsRight = targetWFps - speedAdjustment;
  targetWFpsLeft = targetWFps + speedAdjustment;
}



/***********************************************************************.
 *  turn() Turn with a given radius.
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
    headingError = rangeAngle(gHeading - targetTurnHeading);
  } else {
    xDist = (currentLoc.x - pivotLoc.x);
    yDist = (currentLoc.y - pivotLoc.y);
    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
    targetTurnHeading = rangeAngle(radiusAngle - 90.0);
    headingError = rangeAngle(gHeading - targetTurnHeading);
  }

  float radiusError = sqrt((xDist * xDist) + (yDist * yDist)) - turnRadius;
//  float radiusAdjustment = radiusError * 0.8 * d;
  float radiusAdjustment = radiusError * 0.3 * d;
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
 *  getRouteFps() Limit acceleration to prevent too rapid acceleration--
 *                particularly when starting from zero from the stand.
 *                Also starts deceleration at end of step.
 *                Returns true if is decelerating and reached target fps.
 ************************************************************************/
float getRouteFps() {
  float stopDistance;

  switch (routeCurrentAction) {
    case 'C':
      routeFps = routeScriptFps;
      break;
      
    case 'G':
    case 'T':
      // Check to see if we need to start the decel phase.
      if (isDecelActive && !isDecelPhase) {
        stopDistance = (tpFps * tpFps) / 5.58;
        if (targetDistance <= stopDistance) {
          isDecelPhase = true;
        }
      }

      if (isDecelPhase) {
        routeFps = tpFps - 5.0;
      } else {
        if (routeScriptFps < 0.0) {
          routeFps = routeScriptFps;
        } else { // Going forward.
          if (timeRun < 1000) {  // taking off from stand?
            routeFps = (((float) timeRun) / 200.0) + 0.5; // Accelerate to 5.5 in 1 sec
            if (routeFps > routeScriptFps) routeFps = routeScriptFps;
          } else { // Accelerate to target speed.
//            float inc = (routeScriptFps - tpFps) * 2.0;
//            inc = constrain(inc, -5.0, 5.0);
//            routeFps = tpFps + inc;
              routeFps = routeScriptFps;
          }
        }
      }
      break;


    case 'B':
      if (barrelOneState == SEEK_BARREL) {
        if (seekState == SEEK_SETTLE) routeFps = holdY();
        else routeFps = SEEK_FPS;
      } else if (barrelOneState == PLOT_BARREL) {
        routeFps = holdY();
      } else if (barrelOneState == CIRCLE_BARREL) {  // Circle barrel
        routeFps = TURN_FPS;
      } else { //RECOVER_BARREL
        routeFps = -3.0;
      }
      break;

    case 'P':
      routeFps = holdY();
      break;

    case 'H':
    case 'J':
    default:
      routeFps = routeScriptFps;
      break;
   
  }
  return routeFps;
}



/***********************************************************************.
 *  isTargetReached()  Return true if within 1 ft of target and is
 *                     moving away from the target.
 *                     Return true if at end of decel.
 ***********************************************************************/
boolean isTargetReached() {
  const int RETREAT_TIMES = 10;
  const int RETREAT_DISTANCE = 2.0;
  static double lastTargetDist = 10000.0D;
  static int timesReached = 0;  if (isDecelActive && isDecelPhase) {
    if (wFps <= 0.8) {
      isDecelActive = isDecelPhase = false;
      return true;
    }
  } else {
    setTarget();
    if (targetDistance < RETREAT_DISTANCE) {
      boolean isCloser = ((lastTargetDist - targetDistance) >= 0.0D);
      lastTargetDist = targetDistance;
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
  }
  return false;
}



/***********************************************************************.
 *  coCorrect() Correct the X,Y? & gyroHeading values given the sonar
 *  readings from the hay bales.
 ************************************************************************/
void coCorrect() {
  float sum = 0.0;
  
    // Find the average.
//  for (int i = 0; i < coPtr; i++) sum += coArray[i];
//  float avg = sum / ((float) coPtr);
//  float correction = coDist - avg;\

  // Find the median, Bubble Sort
  boolean swapped;
  while (true) {
    swapped = false;
    for (int i = 1; i < coPtr; i++) {
      float a = coArray[i - 1];
      float b = coArray[i];
      if (a > b) {
        coArray[i - 1] = b;
        coArray[i] = a;
        swapped = true;
      }
    }
    if (!swapped) break;
  }
  float correction = coDist - coArray[2];

  currentLoc.y += correction;
  gHeading += (correction * coHCorrection);
}


/***********************************************************************.
 *  steerHug() 
 ************************************************************************/
void steerHug() {
  static float sonar = 3.0;
  boolean isReading = false;

  // Read the sonar
  if (isHugRight) {
    if (sonarRight > 0.1) {
      sonar = sonarRight;
      sonarRight = 0.0;
      isReading = true;
    }
  } else {
    if (sonarLeft > 0.1) {
      sonar = sonarLeft;
      sonarLeft = 0.0;
      isReading = true;
    }
  }
  if (isReading) {
    isReading = false;
    if (sonar < 7.0) {
      hugSonarArray[hugPtr] = sonar;
      hugDistArray[hugPtr] = ((float) (tickPosition - hugStartTick)) / TICKS_PER_FOOT;
    }
    if (hugPtr < HUG_ARRAY_SIZE) hugPtr++;
  }
  
  float error = sonar - hugWallDistance;
//  float correction = error * 20.0;
  float correction = error * 10.0;
  correction = constrain(correction, -30.0, 30.0);
//  targetBearing = hugHeading + correction;
  targetBearing = hugHeading - correction;
  steerHeading();
}



/***********************************************************************.
 *  hugCorrect()  At end of Hug, correct the heading and X,Y.
 ************************************************************************/
void hugCorrect() {
  float x = currentLoc.x - hugStartLoc.x;
  float y = currentLoc.y - hugStartLoc.y;
  float realizedHeading = atan2(x,y) * RAD_TO_DEG; // Heading from dead reckoning
  float ss = sonarSlope();          // from the sonar
  float sa = atan(ss) * RAD_TO_DEG;
  float correction = hugHeading - realizedHeading - sa;
  float finalHeading = gHeading + correction; 

  sprintf(message1, "startLoc: %5.2f;%5.2f   currentLoc: %5.2f;%5.2f", hugStartLoc.x, hugStartLoc.y, currentLoc.x, currentLoc.y);
  sendBMsg(SEND_MESSAGE, message);  
  sprintf(message2, "hugHeading: %5.2f   hugWallAngle: %5.2f", hugHeading, hugWallAngle);
  sendBMsg(SEND_MESSAGE, message);  
  sprintf(message3, "realizedHeading: %5.2f   sonarSlope %5.2f    sonarAngle: %5.2f", realizedHeading, ss, sa);
  sendBMsg(SEND_MESSAGE, message);  
  sprintf(message4, "correction:   %5.2f    finalHeading: %5.2f  ", correction, finalHeading);
  sendBMsg(SEND_MESSAGE, message);  

  // Set the heading.
  setHeading(finalHeading);

  // Set the cartesian coordinates.
//  currentLoc.x = targetLoc.x;
//  if (abs(sonarRightKeep - hugWallDistance) < 1.0) {
//    currentLoc.y = targetLoc.y - (0.78 * (sonarRightKeep - hugWallDistance));
//  } else {
//    currentLoc.y = targetLoc.y;
//  }
  currentLoc = targetLoc;  // Assume we are now at the correct target.
}



/***********************************************************************.
 *  sonarSlope()  Slope of least squares regression line.  
 ************************************************************************/
float sonarSlope() {
  float df = (float) (hugPtr - 1);
  float sumX = 0.0F;
  float sumY = 0.0F;
  float sumSqX = 0.0F;
  float sumSqY = 0.0F;
  float sumXY = 0.0F;
  float meanX, meanY;
  float diffX, diffY, sigmaX, sigmaY;
  float r, slope, intercept;
  float angleCorrection;

  for (int i = 0; i < hugPtr; i++) sumX += hugDistArray[i];
  for (int i = 0; i < hugPtr; i++) sumY += hugSonarArray[i];
  meanX = sumX / ((float) hugPtr);
  meanY = sumY / ((float) hugPtr);
  for (int i = 0; i < hugPtr; i++) {
    diffX = meanX - hugDistArray[i];
    sumSqX += diffX * diffX;
    diffY = meanY - hugSonarArray[i];
    sumSqY += diffY * diffY;
    sumXY += (diffX * diffY);
  }
  sigmaX = sqrt(sumSqX / df);
  sigmaY = sqrt(sumSqY / df);
  r = (sumXY / (sigmaY * sigmaX)) / df;
  slope = r * (sigmaY / sigmaX);
  double bearing = atan(slope) * RAD_TO_DEG;
  intercept = meanY - (slope * meanX);
//  sprintf(message, "slope: %5.2f   intercept: %5.2f   heading: %5.2f", slope, intercept, bearing);
//  sendBMsg(SEND_MESSAGE, message); 
  return slope;
}

/************************************************************************.
 *  setTarget() Set the new targetBearing and targetDistance from
 *              the currentLoc.
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
 *  Script parsing routines
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

