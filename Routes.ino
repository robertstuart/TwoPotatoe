String loadedRoute[200] = {
  "N Run 5 - place holder",
  "Z   0",
  "S   0",
  "GY  0,30   5",
  "F"
};  

String run6[] = {
  "N Run 6",
  "Z   0",
  "S   0",
  "GY  0,30   6",
  "F"
};  

String run7[] = {
  "N Run 7",
  "Z   0",
  "S   0",
  "GY  0,30   7",
  "F"
};  

String run8[] = {
  "N Run 8",
  "Z   0",
  "S   0",
  "GY  0,30   8",
  "F"
};  

String run9[] = {
  "N Run 9",
  "Z   0",
  "S   0",
  "GY  0,30   9",
  "F"
};  

String run10[] = {
  "N Run 10",
  "Z   0",
  "S   0",
  "GY  0,30   10",
  "GY  0,40   8",
  "F"
};  

String run11[] = {
  "N Run 11",
  "Z   0",
  "S   0",
  "GY  0,30   11",
  "GY  0,40   8",
  "F"
};  

String run12[] = {
  "N Run 12",
  "Z   0",
  "S   0",
  "GY  0,30   12",
  "GY  0,40   8",
  "F"
};  

String run13[] = {
  "N Run 13",
  "Z   0",
  "S   0",
  "GY  0,30   13",
  "GY  0,40   8",
  "F"
};  

String run14[] = {
  "N Run 14",
  "Z   0",
  "S   0",
  "GY  0,30   14",
  "GY  0,40   8",
  "F"
};  

String run15[] = {
  "N Run 15",
  "Z   0",
  "S   0",
  "GY  0,30   15",
  "GY  0,40   8",
  "F"
};

String discombobulate4[] = {
  "N Discombobulate 4",
  "Z   0",
  "S   0",
  "K  0,30   4",
  "F"
};  

String discombobulate5[] = {
  "N Discombobulate 5",
  "Z   0",
  "S   0",
  "K  0,30   5",
  "F"
};  

String discombobulate6[] = {
  "N Discombobulate 6",
  "Z   0",
  "S   0",
  "K  0,30   6",
  "F"
};  

String discombobulate7[] = {
  "N Discombobulate 7",
  "Z   0",
  "S   0",
  "K  0,30   7",
  "F"
};  

String discombobulate8[] = {
  "N Discombobulate 8",
  "Z   0",
  "S   0",
  "K  0,30   8",
  "F"
};  

String discombobulate9[] = {
  "N Discombobulate 9",
  "Z   0",
  "S   0",
  "K   0,30   9",
  "GY  0,40   8",
  "F"
};  

String discombobulate10[] = {
  "N Discombobulate 10",
  "Z   0",
  "S   0",
  "K   0,30   10",
  "GY  0,40   8",
 "F"
};  

String discombobulate11[] = {
  "N Discombobulate 11",
  "Z   0",
  "S   0",
  "K   0,20   11",
  "GY  0,40   8",
  "F"
};  

String shortRoute[] = {
  "N Short - loaded route",
  "Z   0",
  "S	0",
  "GY	  0,48  10",
  "T	116,60  8 10",
  "GX  116,60  10",
  "T   128,-48 8 10",
  "GY  128,-48 10",
  "T	 12,-60 8 10",
  "GX   12,-60 10",
  "T     0,0   8 10",
  "GY    0,0   10",
  "F"
};

String longRoute[] = {
  "N Long - loaded route",
  "Z    0",
  "S	0",
  "E WY	0,49  8",
  "E CY                 1 4  6 6 i",
  "E CY                45 3  7 6 o",
  "GY	 0,49  8",
  "T   165,62  8 7",
  "WX  165,62  8 ",
  "CX                  40 1 10 6",  // CO 1
  "CX                 140 1 10 6",  // CO 2
  "GX  170,62  8",
  "GX  186,69  8",
  "GX  216,62  8",
  "GX  236,69  8",
  "GX  288,62  8",
  "T	298,30  8 7",
  "GY  298,30  8",
  "GY  305,0   8",
  "GY  298,-30 8",
  "GY  298,-49 8",
  "T    10,-59 8 7",
  "GX   10,-59 8",
  "T     0,0   8 7",
  "GY    0,0   8",
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
  loadedRoute, run6, run7, run8, run9, run10, run11, run12, run13, run14, run15,
  discombobulate4, discombobulate5, discombobulate6, discombobulate7, discombobulate8, discombobulate9, discombobulate10, discombobulate11,
  garageLoopSND, garageLoopS, square12, outAndBack12, shortRoute, longRoute};
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
  int nRoutes = (sizeof(routeTable) / sizeof(int)) + 1;
  
  if (increment) routeTablePtr++;
  else routeTablePtr--;
  
  if (routeTablePtr < 0) routeTablePtr = nRoutes -1;
  else if (routeTablePtr >= nRoutes) routeTablePtr = 0;
  
  if (routeTablePtr == nRoutes - 1) currentRoute = loadedRoute;
  else currentRoute = routeTable[routeTablePtr];
  
  routeStepPtr = 0;
  interpretRouteLine(currentRoute[0]);
  sendXMsg(SEND_ROUTE_NAME, routeTitle); 
  sendBMsg(SEND_ROUTE_NAME, routeTitle); 
}

/************************************************************************
 *  resetRoute()
 ************************************************************************/
void resetRoute() {
  routeStepPtr = 0;
  isRouteInProgress = true;
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

