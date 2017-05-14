// AVC Full route, TwoPotatoe
String avcFn[] = {
  "N AVC 1",
  "MR",
  "KR 118,5   -90",
  "HG",
  "GX  10,5    7",       // 1)    Down straightaway away from building
  "T    5,45    5  5",   // 2)                      46
  "GY   5,45    7",      // 3)                      46
  "T   33,47    5  6",   // 4)    Turn toward building  48
  "GX  33,47    7",      // 5)    To second barrel      48
  "GX  54,49    7",      // 6)    To third barrel       50
  "GX  80,47    7",      // 7)    To fourth barrel      48
  "T   89,63    5  7",   // 8)    Turn toward hoop
  "GY  89,63    7",      // 9)    Stage for sonar
//  "CR     70    79",     // 10)    Sonar
  "GY  89,71    7",      // 11)   Sonar run
  "GY  88,80    7",      // 12)   To hoop
  "GY  88,103   7",      // 13)   To end of hoop run     102
  "T  133,108   5  5",   // 14)   Turn toward building   107
  "GX 128,108   5",      //       Stage toward building  107
  "GX 133,108   5",      // 15)   Toward building slow   107
  "T  137,95   5  5",   // 16)   Turn toward ramp    100
  "GY 137,95   6"       // 17)   Stage for sonar     100
//  "CR     93    135",    // 18)   Sonar
  "GY 137,90    6",      // 19)   Sonar run           92
  "GY 135,84    6",      // 20)   Veer in slightly to avoid ramp
  "GY 135,73    6",      // 21)   Straight past ramp  76
  "GY 138,45    7",      // 22)   to pool
  "GY 138,37    5",      // 23)   to hairpins  slow        33
  "T  128,32    5  4",   // 24)   turn away from building  28
  "GX 128,32    5",      // 25)                            28
  "T  125,21    5  4",   // 26)   turn toward start
  "GY 125,21    5",      // 27)
  "T  132,18    5  4",   // 28)   turn toward building     135
  "GX 132,18    5",      // 29)                            135
  "T  137,10    5  4",   // 30)                            140
  "GY 137,10    5",      // 31)                            140
  "T  110,5     5  5",   // 32)
  "GX 122,5     6",
  "GX 110,5     4",      // 33)   End 10' past ramp
  "F"
};


//
//// MLC  TwoPotatoe
//String mlc[] = {
//  "N MLC 1",
//  "MR",
//  "KR 119,0   -90",
//  "HG",
//  "GX  10,5    7",      // Down straightaway away from building
//  "T    5,45    5  5",
//  "GY   5,46    7",
//  "T   30,48    5  6",   // Turn toward building
//  "GX  30,48    7",      // To second barrel
//  "GX  48,52    7",      // To third barrel
//  "GX  67,48    7",      // To fourth barrel
//  "T   76,63    5  7",   // Turn toward hoop
//  "GY  76,63    7",       // Stage for sonar
////  "CR     70    79",     // Sonar
//  "GY  76,71    7",      // Sonar run
//  "GY  74,80    7",      // To hoop
//  "GY  73,108   7",      // To end of hoop run
//  "T  133,115   5  5",   // Turn toward building
//  "GX 128,115   5",      // Toward building
//  "GX 133,115   5",      // Toward building slow
//  "T  140,100   5  5",   // Stage for sonar
////  "CR     97    137",    // Sonar
//  "GY 140,92    6",      // Sonar run
//  "GY 138,84    6",      // Veer in slightly to avoid ramp
//  "GY 138,76    6",      // Straight past ramp
//  "GY 140,50    7",      // to pool
//  "GY 140,42    5",      // to hairpins  slow
//  "T  128,37    4  5",   // turn away from building
//  "GX 128,37    4",
//  "T  125,21    4  5",   // turn toward start
//  "GY 125,21    4",
//  "T  135,18    4  5",   // turn toward building
//  "GX 135,18    4",
//  "T  140,10    4  5",   //
//  "GY 140,10    5",
//  "T  119,5     5  5",   
//  "GX 119,5     5",       // Signal end
//  "GX 109,5     7",      // End 10' past ramp
//  "F"
//};
//
//String houseSonar[] = {
//  "N House Sonar",
//  "ML",
//  "KR 0,0 0",
//  "HG",
//  "GY  0,3 4",
//  "T  16,6.5 4 4",
//  "CL  13.5 8.9",
//  "GX 16,6.5 4",
//  "T  30,2  4 5",
//  "GX 30,2  4",
//  "F"
//};
//
//// From driveway to street and back
//String street1[] = {
//  "N Street 1",
//  "KR    5,30  90",
//  "HG",
//  "GX    8,30  2",    //  
//  "GX   35,27  5",
//  "GX   65,27  6",    //  Enter the street
//  "T    70,50  6 3.5",  //  Turn left up the street
//  "GY   70,50  6",    //  
//  "T    95,75  6 3.5",
//  "GY   95,75  6",    // 11
//  "T    80,88  5 3.5",  // 12  Turn left toward Mensch 
//  "GX   80,88  5",
//  "T    50,30  5 3.5",  // 14 Left to return
//  "GY   50,30  5",
//  "T    35,30  5 3.5",
//  "GX   35,30  5",    // 17 Up curb
//  "GX   10,30  5",
//  "F"
//};


String loadedRoute[200] = {
  "N Place holder",
  "MG",
  "GY  0,2   3",
  "F"
}; 


String *routeTable[] = {
 avcFn
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
//  setSonar(SONAR_BOTH);
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

