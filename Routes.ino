String cal200[] = {
  "N Cal200",
  "KR 0,0 0",
  "HG",
  "GY 0,200 6",
  "F"
};

// AVC Full route, TwoPotatoe
String avcFn[] = {
  "N AVC 1",
  "MR",
  "KR 120,0   -90",
  "HG",
  "GX  10,5    7",      // Down straightaway away from building
  "T    5,46    5  5",
  "GY   5,46    7",
  "T   30,48    5  6",   // Turn toward building
  "GX  30,48    7",      // To second barrel
  "GX  48,52    7",      // To third barrel
  "GX  67,48    7",      // To fourth barrel
  "T   76,63    5  7",   // Turn toward hoop
  "GY  76,63    7",       // Stage for sonar
//  "CR     70    79",     // Sonar
  "GY  76,71    7",      // Sonar run
  "GY  74,80    7",      // To hoop
  "GY  73,102   7",      // To end of hoop run
  "T  133,107   5  5",   // Turn toward building
  "GX 128,107   5",      // Toward building
  "GX 133,107   5",      // Toward building slow
  "T  140,100   5  5",   // Turn toward ramp
  "GY 138,100   6"       // Stage for sonar
//  "CR     93    135",    // Sonar
  "GY 138,92    6",      // Sonar run
  "GY 137,84    6",      // Veer in slightly to avoid ramp
  "GY 137,76    6",      // Straight past ramp
  "GY 140,50    7",      // to pool
  "GY 140,42    5",      // to hairpins  slow
  "T  128,37    5  5",   // turn away from building
  "GX 128,37    5",
  "T  125,21    5  5",   // turn toward start
  "GY 125,21    5",
  "T  135,18    5  5",   // turn toward building
  "GX 135,18    5",
  "T  140,10    5  5",   //
  "GY 140,10    5",
  "T  120,5     5  5",   
  "GX 120,5     5",       // Signal end
//  "GX 110,5     8",      // End 10' past ramp
  "F"
};
// MLC  TwoPotatoe
String mlc[] = {
  "N MLC 1",
  "MR",
  "KR 119,0   -90",
  "HG",
  "GX  10,5    7",      // Down straightaway away from building
  "T    5,45    5  5",
  "GY   5,46    7",
  "T   30,48    5  6",   // Turn toward building
  "GX  30,48    7",      // To second barrel
  "GX  48,52    7",      // To third barrel
  "GX  67,48    7",      // To fourth barrel
  "T   76,63    5  7",   // Turn toward hoop
  "GY  76,63    7",       // Stage for sonar
//  "CR     70    79",     // Sonar
  "GY  76,71    7",      // Sonar run
  "GY  74,80    7",      // To hoop
  "GY  73,108   7",      // To end of hoop run
  "T  133,115   5  5",   // Turn toward building
  "GX 128,115   5",      // Toward building
  "GX 133,115   5",      // Toward building slow
  "T  140,100   5  5",   // Stage for sonar
//  "CR     97    137",    // Sonar
  "GY 140,92    6",      // Sonar run
  "GY 138,84    6",      // Veer in slightly to avoid ramp
  "GY 138,76    6",      // Straight past ramp
  "GY 140,50    7",      // to pool
  "GY 140,42    5",      // to hairpins  slow
  "T  128,37    4  5",   // turn away from building
  "GX 128,37    4",
  "T  125,21    4  5",   // turn toward start
  "GY 125,21    4",
  "T  135,18    4  5",   // turn toward building
  "GX 135,18    4",
  "T  140,10    4  5",   //
  "GY 140,10    5",
  "T  119,5     5  5",   
  "GX 119,5     5",       // Signal end
  "GX 109,5     7",      // End 10' past ramp
  "F"
};

String houseSonar[] = {
  "N House Sonar",
  "ML",
  "KR 0,0 0",
  "HG",
  "GY  0,3 4",
  "T  16,6.5 4 4",
  "CL  13.5 8.9",
  "GX 16,6.5 4",
  "T  30,2  4 5",
  "GX 30,2  4",
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
 avcFn,  street1, houseSonar
};

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
      sprintf(message, "Error step %d!", routeStepPtr - 1);
      sendXMsg(SEND_MESSAGE, message);
      sendBMsg(SEND_MESSAGE, message);
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

