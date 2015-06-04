String loadedRoute[200] = {"N Loaded route",
  "Z         0",
  "WY       0,12   4",        // Step 3
  "CY       0.5    2 4 2.27 i",
  "CY      10.7    2 4 1.72 o",
  "GY       0,12   4",
  "T        10.5,14  4   2", // Turn toward garbage bins
  "WX       10.5,14  4",
  "CX        2.60  1 6 3.8",   
  "CX       10.35  1 6 4.8" ,  
  "GX       10.5,14  4",
  "T        12.5,0    4   2",  // Turn toward street.
  "WY       12.5,0   4",
  "CY        7.3     1  5  2.6",
  "CY        0.0     1  6  3.94",
  "GY       12.5,-0.4    2.8",
  "T        2,-2   4   2",  // Turn toward tools
  "WX       2,-2   4",
  "CX       7.4    0   3  1.6",
  "CX       2.2   0   3  1.1",
  "GX       2,-2   4",
  "T        0,14   5   2",  // Turn toward trebuchet
  "F"
}; 

String simplePOC[] = {
  "N  Simple POC",
  "Z        0",
  "GY	 0,12.2  3",
  "T	10,15.2 4 2.8",
  "T	 6,0    4 2.8",
  "GY   6,0 4",
  "T	-10,-3	4 2.8",
  "T    0,12.2  4 2.8",
  "GY	 0,12.2		3.5",
  "T	10,15.2 4 2.8",
  "T	 6,0    4 2.8",
  "GY   6,0 4",
  "T	-10,-3	4 2.8",
  "T    0,12.2  4 2.8",
  "GY	 0,6.5		3.5",
  "S	80	2",
  "GX	6,7 3",
  "F",
};



String garageLoop[] = {"N Garage loop",
                "Z         0",
                "WY       0,12   4",        // Step 3
                "CY       0.5    2 4 2.27 i",
                "CY      10.7    2 4 1.72 o",
                "GY       0,12   4",
                "T        10.5,14  4   2", // Turn toward garbage bins
                "WX       10.5,14  4",
                "CX        2.60  1 6 3.8",   
                "CX       10.35  1 6 4.8" ,  
                "GX       10.5,14  4",
                "T        12.5,0    4   2",  // Turn toward street.
                "WY       12.5,0   4",
                "CY        7.3     1  5  2.6",
                "CY        0.0     1  6  3.94",
                "GY       12.5,-0.4    2.8",
                "T        2,-2   4   2",  // Turn toward tools
                "WX       2,-2   4",
                "CX       7.4    0   3  1.6",
                "CX       2.2   0   3  1.1",
                "GX       2,-2   4",
                "T        0,14   5   2",  // Turn toward trebuchet
                "F"
}; 

String square12[] = {"N Square 12 ft",
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



String outAndBack12[] = {"N Out and back 12",
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


//String rtA[] = {"N  Brewing Market",
//                "M       0.0", 
//                "Z         0", 
//                "S        90   2",
//                "GX     15,0   3",            // go E to sidewalk
//                "T     17,46   3    2",       // turn toward N
//                "GY    17,46   3",            // to N end of sidewalk
//                "T     48,48   3    2",       // turn E toward lot
//                "GX    48,48   3",            // E into lot
//                "T     56,50   3    2",       // turn North in lot
//                "GY    56,60   3",            // go N a little
//                "T     88,58   3    2",       // turn East at top of lot
//                "GX    88,58   3",            // to East side of lot
//                "T    90,-46   3    2",       // turn toward S
//                "GY   90,-46   3",            // go to SE end of lot
//                "T    52,-48   3    2",       // turn W toward BrewingMarket
//                "GX   52,-48   3",            // Go toward Brewing Market
//                "T     50,46   3    2",       // turn to NW corner of lot
//                "GY    50,46   3",            // go to NW corner of lot
//                "T     19,48   3,   2",       // turn toward Brewing Market
//                "GX    19,48   3",            // go to sidewalk
//                "T      17,2   3    2",       // turn S
//                "GY     17,2   3",            // go S
//                "T       0,0   3    2",
//                "F"    
//}; 

String *routeTable[] = {garageLoop, square12, outAndBack12, simplePOC};
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

