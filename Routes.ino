String loadedRoute[200] = {
  "N Place holder",
  "MG",
  "GY  0,2   3",
  "F"
};  

String stand1[] = {
  "N Stand30",
  "A",
  "MG",
  "GY  0,3   3.0",
  "SYW 4.0",
  "F"
};  

String singleStep[] = {
  "N Single Step",
  "A",
  "MG",
  "GY 0,99  0.0",
  "P",
  "Sy 1.0",
  "F"
};  


String backSteps[] = {
  "N Back Steps",
  "A",
  "MG",
  "GY 0,99  0.0",
  "P",
  "Sy 0.8",
  "P",
  "Sy 0.8",
  "F"
};  


String spSteps[] = {
  "N Skatepark Steps",
  "A",
  "MG",
  "GY 0,99  0.0",
  "P",
  "Sy 0.8",
  "P",
  "Sy 0.8",
  "P",
  "GY 0,10 5.0",
  "F"
};  

String runupJump[] = {
  "N Run Up Jump",
  "A",
  "MG",
  "GY  0,10.0   8",
  "GY  0,11.0   6",
  "GY  0,15.0   15",
  "F"
};  

String smallSquare[] = {
  "N Small Square",
  "A",
  "MG",
  "GY  0,2   2.5",
  "T   4,4   2.5   2.2",
  "GX  4,4   2.5",
  "T   6,0   2.5   2.2",
  "GY  6,0   2.5",
  "T   2,-2  2.5   2.2",
  "GX  2,-2  2.5",
  "T   0,2   2.5   2.2",
  "F"
};  


String shortRoute[] = {
  "N Short - loaded route",
  "Z   0",
  "S	0",
  "GY	  0,6   4",
  "GY	  0,46  8",
  "GY	  0,48  10",
  "T	118,60  8 10",
  "GX   118,60  10",
  "GX   122,60  9",
  "T    134,10  8 10",
  "GY   134,10  10",
  "D    134,-10 7",
  "GY   134,-44 10",
  "GY   134,-48 8",
  "T	 12,-60 8 10",
  "GX   16,-60 10",
  "GX   12,-60 8",
  "T     0,-4  8 10",
  "GY    0,-4  10",
  "GY    0,4   4",
  "F"
};

String longRoute[] = {
  "N Long - loaded route",
  "Z    0",
  "S	0",
//  "E WY	0,49  8",
//  "E CY                 1 4  6 6",
//  "E CY                45 3  7 6 o",
  "GY	 0,4  4",
  "GY	 0,49  10",
  "T   155,62  8 7",
// "WX  165,62  8 ",
//  "CX                  31 1 10 6",  // CO 1
//  "CX                 102 1 10 6",  // CO 2
//  "CX                 150 1 10 6",  // CO 2
  "GX  170,62  10",
  "GX  186,69  10",
  "GX  216,62  10",
  "GX  236,69  10",
  "GX  278,62  9",
  "T   288,-52  8 7",
//  "GY  288,30  8",
//  "GY  305,0   8",
//  "GY  298,-30 8",
  "GY  288,-52 9",
  "T    10,-62 8 7",
  "GX   12,-62 10",
  "GX   10,-62 10",
  "T     0,0   8 7",
  "GY    0,-5   8",
  "GY    0,10  4",
  "F"
};


String garageLoopSND[] = {"N Garage Sonar NoDis",
  "Z     0",
  "S     0",
  "WY          0,12    3",                            // Step 3
  "CY                       0.5    2  4 2.6 i",
  "CY                      10.7    2 4 1.72 o",
  "GY          0,12    3",
  "T          11,14    3   2",                      // Turn toward garbage bins
  "WX         11,14    3",
  "CX                       2.60  1 6 3.8",   
  "CX                       9.0   1 6 3.8" ,  
  "GX         11,14    3",
  "T          13,-0.5  3   2",                      // Turn toward street.
  "WY         13,-0.5  3",
  "CY                      10     1  5  3.9",
  "CY                       0     1  6  4.5",
  "GY         13,-0.5  3",
  "T           2,-2.5  3   2",                      // Turn toward tools
  "WX          2,-2.5  3",
  "CX                         8    0   3  2.3",
  "GX          2,-2.5  3",
  "T           0,12    3   2",                      // Turn toward trebuchet
  "F"
}; 
    



String garageLoopS[] = {"N Garage loop Sonar",
  "Z     0",
  "S     0",
  "WY         0,12  4",                            // Step 3
  "CY                       0.5    2  4 2.6 i",
  "CY                      10.7    2 4 1.72 o",
  "GY          0,12   4",
  "T          11,14   4   2",                      // Turn toward garbage bins
  "WX         11,14   4",
  "CX                       2.60  1 6 3.8",   
  "CX                       9.0   1 6 3.8" ,  
  "GX         11,14  4",
  "T          13,-.5    4   2",                   // Turn toward street.
  "GY         13,8     4.5",
  "D          13,2     4",
  "E WY       12.5,0   3",
  "E CY        9.0     1  5  3.9",
  "E CY        0.0     1  6  4.5",
  "GY         13,-0.5  3.5",
  "T           2,-2.5  4   2",  // Turn toward tools
  "E WX       2,-2   4",
  "E CX       8.0    0   3  2.3",
  "E CX       2.2   0   3   2.6",
  "GX        2,-2.5   4",
  "T         0,14   5   2",  // Turn toward trebuchet
  "F"
}; 
    

String garageLoopNS[] = {"N Garage loop no Sonar",
  "Z     0",
  "S     0",
  "GY     0,12    4",
  "T     11,14    4   2", // Turn toward garbage bins
  "GX    11,14    4",
  "T     13,-0.5  4   2",  // Turn toward street.
  "GY    13,8     4.5",
  "D     13,2     4",
  "GY    13,-0.5  3.5",
  "T      2,-2.5  4   2",  // Turn toward tools
  "GX     2,-2.5  4",
  "T      0,12    5   2",  // Turn toward trebuchet
  "F"
}; 

    
String square12[] = {
  "N Square 12 ft",
  "Z         0",
  "GY        0,8   3",
  "T        10,10  3   2", 
  "GX       10,10  3",
  "T        12,0    3   2", 
  "GY       12,0    2.8",
  "T        2,-2   4   2", 
  "GX       2,-2   4",
  "T        0,8   5   2",  
  "GY        0,8   3",
  "T        10,10  3   2", 
  "GX       10,10  3",
  "T        12,0    3   2", 
  "GY       12,0    2.8",
  "T        2,-2   4   2", 
  "GX       2,-2   4",
  "T        0,8   5   2",  
  "GY        0,8   3",
  "T        10,10  3   2", 
  "GX       10,10  3",
  "T        12,0    3   2", 
  "GY       12,0    2.8",
  "T        2,-2   4   2", 
  "GX       2,-2   4",
  "T        0,8   5   2",  
  "F"
}; 



String outAndBack12[] = {
  "N Out and back 12",
  "Z         0",
  "GY        0,8   3",
  "T        10,10  3   2", 
  "T        4,0  3   2", 
  "GY       4,0  3",
  "T        -10,-2   3     2", 
  "T        0,8      3     2",
  "GY        0,8   3",
  "T        10,10  3   2", 
  "T        4,0  3   2", 
  "GY       4,0  3",
  "T        -10,-2    3   2", 
  "T         0,8   3",
  "GY        0,8   3",
  "T        10,10  3   2", 
  "T        4,0  3   2", 
  "GY       4,0  3",
  "T        -10,-2    3   2", 
  "T        0,8    3",
  "F"
}; 


String rtA[] = {
  "N  Brewing Market",
  "M       0.0", 
  "Z         0", 
  "S        90   2",
  "GX     15,0   3",            // go E to sidewalk
  "T     17,46   3    2",       // turn toward N
  "GY    17,46   3",            // to N end of sidewalk
  "T     48,48   3    2",       // turn E toward lot
  "GX    48,48   3",            // E into lot
  "T     56,50   3    2",       // turn North in lot
  "GY    56,60   3",            // go N a little
  "T     88,58   3    2",       // turn East at top of lot
  "GX    88,58   3",            // to East side of lot
  "T    90,-46   3    2",       // turn toward S
  "GY   90,-46   3",            // go to SE end of lot
  "T    52,-48   3    2",       // turn W toward BrewingMarket
  "GX   52,-48   3",            // Go toward Brewing Market
  "T     50,46   3    2",       // turn to NW corner of lot
  "GY    50,46   3",            // go to NW corner of lot
  "T     19,48   3,   2",       // turn toward Brewing Market
  "GX    19,48   3",            // go to sidewalk
  "T      17,2   3    2",       // turn S
  "GY     17,2   3",            // go S
  "T       0,0   3    2",
  "F"    
}; 

String *routeTable[] = {
  loadedRoute, singleStep, backSteps, stand1, smallSquare, runupJump,};
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
  sendXMsg(SEND_ROUTE_NAME, routeTitle); 
  sendBMsg(SEND_ROUTE_NAME, routeTitle); 
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
      sprintf(message, "Error step %d!", routeStepPtr - 1); isNewMessage = true;
      return;
    }
    if (!isRouteInProgress) break;
  }
  // It made it here.  Therefore run it.
  routeStepPtr = 0;
  interpretRouteLine(getNextStepString()); // Load the first line.
  isRouteInProgress = true;
  resetNavigation('M', 0);
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

