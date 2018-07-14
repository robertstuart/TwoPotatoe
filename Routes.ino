
String avc2017[] =   {  // hoop + ramp
  "N AVC 2017",
  "KR   63.4,57.7             129        ",
  "G   117,11       8               ",
  "D                 0               ",
  "T   165,32       8    29       0 ",  // Turn toward barrels
  "B   165.5          53               ",  // Barrels
  "G   154,65       8               ",
  "T   131,65        8    18    -129 ",  // approach straightaway
  "G   124,60        8               ",  // approach sonar run
//  "G   101,40.5        6               ",  // sonar run, swap with HL
  "HL  101,40.5        6     3.0  -129 ",  // sonar run, swap with G
  "G    68,35       6               ",  // approach hoop
  "G    60,21        6               ",  // through hoop
  "G    42,14       8               ",  // Approach turn
  "D                 0               ",
  "T    11,51        9    32      -7 ",  // Turn toward pedestrian

  "P                                 ",
  "T    58,64       9    37     129 ",
  "G    75,51        4               ",
  "F"
};

String avc2017_hr_old[] =   {  // hoop + ramp
  "N AVC 2017-HR",
  "KR   65,59             129        ",
  "G   118,11       12               ",
  "D                 0               ",
  "T   166,32       12    29       0 ",  // Turn toward barrels
  "B   167          53               ",  // Barrels
  "G   145,70       10               ",
  "T   120,70        8    19    -129 ",  // approach straightaway
  "HR   96,52        6     3.0  -129 ",
  "J  -129           4.0   2.5       ",  // Jump
  "G    79,38        5               ",  // Approach hoop
  "T    76,32        5    10    -173 ",  // Turn into hoop
  "T    70,20        7    17    -129 ",  // Turn out of hoop
  "G    59,12       12               ",
  "D                 0               ",
  "T     8,34       12    32      -7 ",  // Turn toward pedestrian
  "P                                 ",
  "T    58,64       12    33     129 ",
  "G    75,51        4               ",
  "F"
};

// inside
String hug[] = { 
  "N Hug",
  "KR  44,32     -175                 ",
  "G   44,25        3                 ", 
  "G   40,18.5      3                 ",
  "HL  33.7,13.2    3     1.8    -129 ",  // sonar run, swap with G
//  "G    33.7,13.2   3               ",  // Swap with HL
  "G   24.1,9.1        3                 ",  //  Final target
  "F"
};

//// outside
//String hug[] = { 
//  "N Hug",
//  "KR  131,76     -178             ",
//  "G   131,67        6               ",  // approach sonar run
//  "G   101,42  7        6              ",
////  "HL  101,42        6     3.0  -129 ",  // sonar run, swap with G
//  "G    76,36       6               ",  // approach hoop
//  "F"
//};
//
String jump1[] = { 
  "N Jump 1",
  "KR  0,0     0          ",
  "G   0,15    4          ",
  "J     0     4       2.5",
  "F"
};

String barrel[] = {
  "N Barrel Test",
  "KR   0,0    0",
  "D           0",
  "G   0,3     4",
  "B   1      25",
  "F"
};

String pedestrian1[] = {
  "N Pedestrian Test 1",
  "KR   0,0    0",
  "D           0",
  "G   0,10    5",
  "P",
  "G  0,20     5",
  "F"
};

String avc10[] =   {  // 1/10 scale
  "N AVC 1/10",
  "KR  5.9,4.8     127          ",
  "G  11.2,0.8       3          ",
  "T  16.0,3.0       3   3     0",
  "T  11.2,5.4       3   3  -127",
  "G   4.8,0.8       3          ",
  "T   0.0,3.0       3   3     0",
  "T   4.8,5.4       3   3   127",
  "F"
};

String gyroTest[] =   {  // run to hoop
  "N Gyro Test",
  "KR   65,59             129        ",
  "G   117,11       12              ",
  "D                       0        ",
  "T   167,32       12    29      0 ",  // Turn toward barrels
  "B   167          52              ",  // Barrels
  "G   140,67       10              ",  // Leave barrels
  "T   122,68        8    19     -129 ",  
//  "C  -129         8     2.4   0.0",
  "G    79,38        5              ",  // Approach hoop
  "T    76,32        5    10   -173 ",  // Turn into hoop
  "F"
};

String avc2017_h[] =   {  // hoop, no ramp
  "N AVC 2017-H",
  "KR  65,59             129        ",
  "G  118,11        12              ",
  "D                       0        ",
  "T   166,32       12    29      0 ",  // Turn toward barrels
  "B   166          52              ",  // Barrels
  "T   127,64       10    25   -129 ",  // Leave barrels
  "G   112,52        8              ",
  "G    81,37        6              ",  // Turn toward hoop
  "T    76,31        4     8   -173 ",  // Turn into hoop
  "T    69,20        5    17   -129 ",  // Turn out of hoop
  "G    59,12       12              ",
  "D                 0              ",
  "T     8,34       12    32     -7 ",  // Turn toward pedestrian
  "P                                ",
  "T    53,69       12    32    129 ",
  "G    75,51        4              ",
  "F"
};

String testRun1[] = { 
  "N Test Run 1",
  "KR   0,0    0         ",
  "G    0,3    3         ",
  "T    3,5    3   2   90",
  "G    5,5    3         ",
  "F"
};

String loadedRoute[200]; 

String *routeTable[] = {
  avc2017
};

int routeTablePtr = 0;
boolean isLoadedRouteValid = true;

String *currentRoute = routeTable[routeTablePtr];

String getNextStepString() {
  return currentRoute[routeStepPtr++];
}



/***********************************************************************.
 *  setRoute()
 ***********************************************************************/
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

/***********************************************************************.
 *  startRoute()
 ***********************************************************************/
void startRoute() {
  isRunReady = false;
  routeStepPtr = 0;
  isRouteInProgress = true;
  // Run through it to see if it compiles
  while (true) {
    if (!interpretRouteLine(getNextStepString())) {
      isRouteInProgress = false;
      sprintf(message, "Error step %d!", routeStepPtr);
      sendXMsg(SEND_MESSAGE, message);
      sendBMsg(SEND_MESSAGE, message);
      Serial.println(message);
      return;
    }
    if (!isRouteInProgress) break;
  }
  // It made it here.  Therefore run it.
  setGyroDrift();
  routeStepPtr = 0;
  isDecelActive = isDecelPhase = false;
  interpretRouteLine(getNextStepString()); // Load the first line.
  isRouteInProgress = true;
//  setSonar("lfR");
  setHeading(0.0D);
  resetTicks();
  currentLoc.x = 0.0D;
  currentLoc.y = 0.0D;
  coSetLoc = currentLoc;
  timeStart = timeMilliseconds;
  timeRun = 0;
  isBackLeft = false;
//  isUpError = false;
}



/***********************************************************************.
 *  stopRoute()
 ***********************************************************************/
void stopRoute() {
  isRouteInProgress = false;
  setHeading(0.0);
//  setSonar(SONAR_BOTH);
}



/***********************************************************************.
 *  loadRouteLine()  Call when route string arrives from PC
 ***********************************************************************/
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

