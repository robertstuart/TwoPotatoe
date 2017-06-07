// Initial test of TwoPotatoe for 2017
String barrel[] = {
  "N Barrel Test 1",
  "KR   0,0    0",
  "D           0",
  "G   0,2     2",
  "B   1      18",
  "F"
};

String pedestrian1[] = {
  "N Pedestrian Test 1",
  "KR   0,0    0",
  "D           0",
  "G   0,3     3",
  "P",
  "G  0,12      5",
  "F"
};

String avc10[] =   {  // 1/10 scale
  "N AVC 1/10",
  "KR  5.9,4.8     127          ",
  "G  11.2,0.8       5          ",
  "T  16.0,3.0       5   3     0",
  "T  11.2,5.4       5   3  -127",
  "G   4.8,0.8       5          ",
  "T   0.0,3.0       5   3     0",
  "T   4.8,5.4       5   3   127",
  "F"
};

String avc2017[] =   {  // 
  "N AVC 2017",
  "KR  61.5,54.5   129             ",
  "G    70,47.5      8             ",
  "G    77,42.3      8             ",
  "G 118.5,9.8       8             ",
  "T   166,32        8   29       0",  // Turn towards barrels
  "G   166,51.7      8             ",  // Barrels
  "T 129.5,66.7      8   20    -129",
  "G    59,12        8             ",
  "T     8,34        8   32.7    -7",
  "G     8,44        8             ",
  "T    59,64        8   33      129",
  "G    75,51        8              ",
  "F"
};

String testRun1[] = { 
  "N Test Run 1",
  "KR  0,0     0          ",
  "G   0,3     3          ",
  "T    3,5    3   2   90",
  "G    5,5    3          ",
  "F"
};

String loadedRoute[200]; 

String *routeTable[] = {
  barrel,
//  pedestrian1,
//  avc2017,
//  avc10,
  testRun1
//  testRun3
//  testRun4
};

int routeTablePtr = 0;
boolean isLoadedRouteValid = true;

String *currentRoute = routeTable[routeTablePtr];

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
  isRunReady = false;
  routeStepPtr = 0;
  isRouteInProgress = true;
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
  isDecelActive = isDecelPhase = false;
  interpretRouteLine(getNextStepString()); // Load the first line.
  isRouteInProgress = true;
  coPtr = coEnd = 0;
  setSonar("lFr");
  setHeading(0.0D);
  resetTicks();
  currentLoc.x = 0.0D;
  currentLoc.y = 0.0D;
  coSetLoc = currentLoc;
  timeStart = timeMilliseconds;
  timeRun = 0;
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
    sprintf(message, "Error on line: %d", loadStepPtr +2);
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

