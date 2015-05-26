 
#define STEP_ERROR -42.42
int originalStepStringPtr = 0;
String stepString = "";
int numLen = 0;

/************************************************************************
 *  Route() called every loop (400/sec)
 *          1. Check if target reached.
 *          2. Set currentLoc.
 *          3. Adjust steering.
 ************************************************************************/
void route() {
  boolean isNewRouteStep = false;
  setTargetPosition();
  
  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'N':
    case 'M':
    case 'Z':
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
    case 'x': // Moving toward X axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x; 
      routeCoDistance = routeCoDistanceXY - currentMapLoc.x;
      if (isRouteTargetIncreasing) {
        if (routeCoDistance <= 0.0) {
          isNewRouteStep = true;
          readSonar();
        }
      }
      else {
        if (routeCoDistance >= 0.0) {
          isNewRouteStep = true;
          readSonar();
        }
      }
      steerHeading();
      break;
    case 'X': // Move toward X axis
      routeTargetXYDistance = routeTargetXY - currentMapLoc.x; 
      if (isRouteTargetIncreasing) {
        if (routeTargetXYDistance <= 0.0) isNewRouteStep = true;
      }
      else {
        if (routeTargetXYDistance >= 0.0) isNewRouteStep = true;
      }
      steerHeading();
      break;
    case 'y': // Moving toward Y axis & waiting to take a sonar reading
      routeTargetXYDistance = routeTargetXY - currentMapLoc.y; 
      routeCoDistance = routeCoDistanceXY - currentMapLoc.y;
      if (isRouteTargetIncreasing) {
        if (routeCoDistance <= 0.0) { 
          isNewRouteStep = true;
          readSonar();
        }
      }
      else {
        if (routeCoDistance >= 0.0) {
          isNewRouteStep = true;
          readSonar();
        }
      }
      steerHeading();
      break;
    case 'Y': // Move toward Y axis
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
    interpretRouteLine();
  }
}



/************************************************************************
 *  readSonar() 
 ************************************************************************/
 void readSonar() {
  double diff = 0.0;
  sonarRight = ((double) analogRead(SONAR_RIGHT_AN)) * SONAR_SENS; // to feet
  if ((sonarRight > routeSonarMin) && (sonarRight < routeSonarMax)) {
    diff = routeSonarDist - sonarRight;
//    if (routeCurrentAction == 'x') currentMapLoc.y += diff;
//    else currentMapLoc.x += diff;
  }
}


/************************************************************************
 *  steerHeading() Find the correct heading to the target and adjust the 
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 ************************************************************************/
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
 *  setPosition() Set the new XY position given the distance and
 *                orientation traveled since the last call.
 ************************************************************************/
void setTargetPosition() {
  // Set the bearing to the target
  double x =  routeTargetLoc.x - currentMapLoc.x;
  double y = routeTargetLoc.y - currentMapLoc.y;
  routeTargetBearing = atan2(x,y) * RAD_TO_DEG;
}



/************************************************************************
 *  resetRoute() 
 ************************************************************************/
void resetRoute() {
  routeStepPtr = 0;
  isRouteInProgress = true;
  // Run through it to see if it compiles
  while (true) {
    if (!interpretRouteLine()) {
      isRouteInProgress = false;
      sprintf(message, "Route error!"); isNewMessage = true;
      return;
    }
    if (!isRouteInProgress) break;
  }
  // It made it here.  Therefore run it.
  routeStepPtr = 0;
  interpretRouteLine();
  isRouteInProgress = true;
}



/************************************************************************
 *  interpretRouteLine()  
 ************************************************************************/
boolean interpretRouteLine() {
  double retDbl;
  int retInt;
  char charX;

  stepString = rtA[routeStepPtr++];
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
      
    case 'S':
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
      
    case 'C':
      charX = stepString.charAt(0);
      if (charX != 'X') routeCurrentAction = 'x';
      else if (charX != 'Y') routeCurrentAction = 'y';
      else return false;
      stepString = stepString.substring(1);

      routeCoDistanceXY = readNum();
      Serial.print(routeCoDistanceXY);
      if (routeCoDistanceXY == STEP_ERROR) return false;
      
      routeSonarMin = readNum();
      Serial.print(routeSonarMin);
      if (routeSonarMin == STEP_ERROR) return false;
      
      routeSonarMax = readNum();
      Serial.print(routeSonarMax);
      if (routeSonarMax == STEP_ERROR) return false;
      
      routeSonarDist = readNum();
      Serial.print(routeSonarDist);
      if (routeSonarDist == STEP_ERROR) return false;
      break;
      
    case 'T':
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
      
      setTargetPosition();
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
  sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction); isNewMessage = true;
  return true;  
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
