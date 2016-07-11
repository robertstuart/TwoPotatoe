// AVC Full route, no ramp
String avcFn[] = {
  "N Full miss ramp",
  "KR 119,5   -90",
  "HG",
  "GX  10,5    7",      // Down straightaway away from building
  "T    5,45    5  5",
  "GY   5,45    7",
  "T   30,48    5  6",   // Turn toward building
  "GX  30,48    7",      // To second barrel
  "GX  48,52    7",      // To third barrel
  "GX  64,48    7",      // To fourth barrel
  "T   75,75    5",      // Turn toward hoop
  "GY  75,75    7",      // 3' from bales for sonar measurement
  "GY  73,84    7",      // To hoop
  "GY  73,108   7",      // To end of hoop run
  "T  135,113   5  6",
  "GX 135,113   7",      // Toward building
  "T  138,96    5  3",   // close to bales
  "GY 138,96    7",      // 3' from bales on R for measurement
  "GY 137,86    6",      // past bales
  "GY 140,42    7",      // to hairpins
  "T  130,37    5  3",   // turn away from building
  "GX 130,37    5",
  "T  125,23    5  3",   // turn toward start
  "GY 125,23    5",
  "T  135,18    5  3",   // turn toward building
  "GX 135,18    5",
  "T  140,10    5  3",   //
  "GY 140,10    5",
  "T  110,5     5  5",
  "GX 110,5     7",
  "F"
};

// From driveway to street and back
String street1[] = {
  "N Street 1",
  "KR    5,30  90",
  "HG",
  "GX    8,30  2",    //  
  "GX   35,27  5",
  "GX   65,27  6",    //  Enter the street
  "T    70,50  6 3.5",  //  Turn left up the street
  "GY   70,50  6",    //  
  "T    95,75  6 3.5",
  "GY   95,75  6",    // 11
  "T    80,88  5 3.5",  // 12  Turn left toward Mensch 
  "GX   80,88  5",
  "T    50,30  5 3.5",  // 14 Left to return
  "GY   50,30  5",
  "T    35,30  5 3.5",
  "GX   35,30  5",    // 17 Up curb
  "GX   10,30  5",
  "F"
};


String loadedRoute[200] = {
  "N Place holder",
  "MG",
  "GY  0,2   3",
  "F"
}; 


String *routeTable[] = {
   avcFn};

int routeTablePtr = 0;
boolean isLoadedRouteValid = true;

String *currentRoute = routeTable[0   ];
//String *currentRoute[] = routeTable[0];

String getNextStepString() {
  return currentRoute[routeStepPtr++];
}



/************************************************************************
 *  setRoute()
 ************************************************************************/
void setRoute(boolean increment) {
  int nRoutes = (sizeof(routeTable) / sizeof(int));
  
  if (increment) {
    routeTablePtr++;
    if (routeTablePtr >= nRoutes) routeTablePtr = 0;
  }
  else {
    routeTablePtr--;
    if (routeTablePtr < 0) routeTablePtr = nRoutes - 1;
  }
  currentRoute = routeTable[routeTablePtr];
  
  routeStepPtr = 0;
  interpretRouteLine(currentRoute[0]);
  sendXMsg(SEND_MESSAGE, routeTitle); 
  sendBMsg(SEND_MESSAGE, routeTitle); 
}

/************************************************************************
 *  startRoute()
 ************************************************************************/
void startRoute() {
  routeStepPtr = 0;
  isRouteInProgress = true;
  isEsReceived = false;
  // Run through it to see if it compiles
  while (true) {
    if (!interpretRouteLine(getNextStepString())) {
      isRouteInProgress = false;
      sendXMsg(SEND_MESSAGE, "Error step %d!");
      sendBMsg(SEND_MESSAGE, "Error step %d!");
//      sprintf(message, "Error step %d!", routeStepPtr - 1); isNewMessage = true;
      return;
    }
    if (!isRouteInProgress) break;
  }
  // It made it here.  Therefore run it.
  routeStepPtr = 0;
  interpretRouteLine(getNextStepString()); // Load the first line.
  isRouteInProgress = true;
  coPtr = coEnd = 0;
  
  setHeading(0.0D);
  resetTicks();
  currentMapLoc.x = 0.0D;
  currentMapLoc.y = 0.0D;
  coSetLoc = currentMapLoc;
}



/************************************************************************
 *  stopRoute()
 ************************************************************************/
void stopRoute() {
  isRouteInProgress = false;
  setHeading(0.0);
  setSonar(SONAR_BOTH);
}



/************************************************************************
 *  loadRouteLine()  Call when route string arrives from PC
 ************************************************************************/
void loadRouteLine(String routeLine) {
  static int loadStepPtr = 0;
  Serial.print("S: "); Serial.print(loadStepPtr); Serial.print("\t");
//Serial.println(routeLine);
  boolean ret = interpretRouteLine(routeLine);
  if (ret == false) {
    sprintf(message, "Error on line: %d", loadStepPtr +1);
    sendBMsg(SEND_MESSAGE, message); 
    loadStepPtr = 0;
    isLoadedRouteValid = false;
    return;
  }
  
  if (routeCurrentAction == 'N') {
    loadStepPtr = 0;
    isLoadedRouteValid = false;
  } else if (routeCurrentAction == 'F') {
//    interpretRouteLine(loadedRoute[0]);
//    if (routeCurrentAction == 'N') {
//      isLoadedRouteValid = true;
//    }
//    isLoadedRouteValid = true;
  } else {
    isLoadedRouteValid = false;
  }
  loadedRoute[loadStepPtr++] = routeLine;
}

