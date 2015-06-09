const int B_BUFFER_SIZE = 100;
char msgStrB[B_BUFFER_SIZE];
int msgStrPtrB = 0;
int msgCmdB = 0;
boolean isMessageInProgressB = false;
char msgStrX[B_BUFFER_SIZE];
int msgStrPtrX = 0;
int msgCmdX = 0;
boolean isMessageInProgressX = false;

void   readBluetooth() {
  while (BLUE_SER.available()) {
    byte b = BLUE_SER.read();
    if (b >= 128) {
      msgStrPtrB = 0;
      msgCmdB = b;
      isMessageInProgressB = true;
    }
    else {
      if (isMessageInProgressB) {
        if (msgStrPtrB >= B_BUFFER_SIZE) {
          isMessageInProgressB = false;
        } else if (b == 0) {
          msgStrX[msgStrPtrX] = 0;
          doMsg(msgCmdB, msgStrB, msgStrPtrB, false);
        } 
        else {
          msgStrB[msgStrPtrB++] = b;
        }
      }
    }
  }
}


void   readXBee() {
  while (XBEE_SER.available()) {
    byte b = XBEE_SER.read();
    if (b >= 128) {
      msgStrPtrX = 0;
      msgCmdX = b;
      isMessageInProgressX = true;
    }
    else {
      if (isMessageInProgressX) {
        if (msgStrPtrX >= B_BUFFER_SIZE) {
          isMessageInProgressX = false;
        } else if (b == 0) {
          msgStrX[msgStrPtrX] = 0;
          doMsg(msgCmdX, msgStrX, msgStrPtrX, true);
        } 
        else {
          msgStrX[msgStrPtrX++] = b;
        }
      }
    }
  }
}



void doMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal;
  float floatVal;
  boolean booleanVal;
  int x = 0;
  String ss;
  
  tHc = timeMilliseconds;
  msgStr[count] = 0; // Just to be sure.
  
  switch(cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) >0) {
        hcX = floatVal; 
        pcX = 0.0;
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        hcY = floatVal;
        pcY = 0.0;
      }
      break;
    case RCV_RUN:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isRunReady = (x == 0) ? false : true;
      }
      break;
    case RCV_LIGHTS:
      if (sscanf(msgStr, "%d", &x) > 0) {
        x = (x == 0) ? LOW : HIGH;
        digitalWrite(RIGHT_HL_PIN, x);
        digitalWrite(LEFT_HL_PIN, x);
        digitalWrite(REAR_TL_PIN, x);
//        sprintf(message, "Lights: %d", x); isNewMessage = true;
      }
      break;
    case RCV_ROUTE:
      if (sscanf(msgStr, "%d", &x) > 0) {
        if (x == 1) {
          isRouteInProgress = true;
          resetRoute();
        }
        else {
          isRouteInProgress = false;
        }
      }
      break;
    case RCV_ROUTE_ES:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isEsReceived = true;
      }
      break;
    case RCV_DUMP_START:
      sendDumpData();
      break;
    case RCV_HOLD_HEADING:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isHoldHeading = (x == 0) ? false : true;
      }
      break;
    case RCV_HOLD_FPS:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isHoldFps = (x == 0) ? false : true;
      }
      break;
    case RCV_GYRO_STEER:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isGyroSteer = (x == 0) ? false : true;
      }
      break;
    case RCV_SET_ROUTE:      // 0 to decrease, 1 to increase
      if (sscanf(msgStr, "%d", &x) > 0) {
        setRoute((x == 0) ? false : true);
      }
      break;
    case RCV_ROUTE_DATA: 
      loadRouteLine(String(msgStr));
      break;
    case RCV_DELETE_ROUTE:
      break;
    case RCV_MODE:
      break;
    case RCV_Y:
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).y = floatVal;
      } 
      break;
    case RCV_Z:
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).z = floatVal;
      } 
      break;
    case RCV_RESET_NAV:
      resetNavigation(0.0);
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}



void setValSet(int newValSet) {
  if (newValSet == vSetStatus) {
    return;
  }
  vSetStatus = newValSet;
  if (mode == MODE_TP4) {
    switch (vSetStatus) {
    case VAL_SET_A:
      currentValSet = &tp4A;
      break;
    case VAL_SET_B:
      currentValSet = &tp4B;
      break;
    case VAL_SET_C:
      currentValSet = &tp4C;
      break;
    default:  
      currentValSet = &tp4A;
      break;
    }
  }
}


// Start delimiter, count MSB, count LSB, API id, Frame id, dest msb, dest lsb, options
byte transmitBuffer[120] = {0x7E, 0, 0, 0x01, 0, 0xFF, 0xFF, 0x04}; 
byte xbeeBuffer[130] = {0x7E, 0, 0};
int transmitBufferLength = 0;
int transmitBufferPtr = 0;
unsigned long transmitNextWriteTime = 0UL;
int dumpPtr, dumpEnd;

/*********************************************************
 * SendStatus??() 
 *********************************************************/
void sendStatusBluePc() {
  sendBMsg(SEND_FPS, 2, wheelSpeedFps);
  sendBMsg(SEND_HEADING, 1, magHeading);
  sendBMsg(SEND_SONAR, 2, sonarRight);
  sendBMsg(SEND_ROUTE_STEP, routeStepPtr); // integer
  sendBMsg(SEND_BATT, battVolt); // integer
  if (isNewMessage) {
    sendBMsg(SEND_MESSAGE, message); 
  }
  sendBMsg(SEND_STATE, getState()); // MUST BE LAST MESSAGE!
}

void sendStatusXBeeHc() {
  sendXMsg(SEND_FPS, 2, wheelSpeedFps);
  sendXMsg(SEND_HEADING, 1, magHeading);
  sendXMsg(SEND_SONAR, 2, sonarRight);
  sendXMsg(SEND_ROUTE_STEP, routeStepPtr); // integer
  sendXMsg(SEND_BATT, battVolt); // integer
  sendXMsg(SEND_X, 2, currentMapLoc.x); // float
  sendXMsg(SEND_Y, 2, currentMapLoc.y); // float 
  if (isNewMessage) {
    sendXMsg(SEND_MESSAGE, message); 
  }
  sendXMsg(SEND_STATE, getState()); // MUST BE LAST MESSAGE!
}



int getState() {
  int statusInt = 0;
  if (isRunning)          statusInt += 1;
  if (isRunReady)         statusInt += 2;
  if (isUpright)          statusInt += 4;
  if (isOnGround)         statusInt += 8;
  if (isHcActive)         statusInt += 16;
  if (isPcActive)         statusInt += 32;
  if (isRouteInProgress)  statusInt += 64;
  if (isDumpingData)      statusInt += 128;
  if (isHoldHeading)      statusInt += 256;
  if (isHoldFps)          statusInt += 512;
  if (isGyroSteer)        statusInt += 1024;
  return statusInt;
}


/*********************************************************
 * dumpData() Set up to start a data dump.
 *********************************************************/
void sendDumpData() {
  dumpEnd = dumpPtr =  dataArrayPtr;
  isDumpingData = true;
}



/*********************************************************
 * dumpData()
 *********************************************************/
void dumpData() {
  BLUE_SER.write(SEND_DUMP_DATA);
  dumpPtr = (dumpPtr + 1) %  DATA_ARRAY_SIZE;
  if (dumpPtr != dumpEnd) {
    BLUE_SER.print(aArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(bArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(cArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(dArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(eArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(fArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(gArray[dumpPtr]);
  }
  else {
    BLUE_SER.print("12345678");
    isDumpingData = false;
  }
  BLUE_SER.write((byte) 0);
}



/*********************************************************
 * send?Msg()
 *********************************************************/
void sendBMsg(int cmd, int precision, double val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val, precision);
  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, int val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, String val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

void sendXMsg(int cmd, int precision, double val) {
  XBEE_SER.write(cmd);
  XBEE_SER.print(val, precision);
  XBEE_SER.write((byte) 0);
}

void sendXMsg(int cmd, int val) {
  XBEE_SER.write(cmd);
  XBEE_SER.print(val);
  XBEE_SER.write((byte) 0);
}

void sendXMsg(int cmd, String val) {
  XBEE_SER.write(cmd);
  XBEE_SER.print(val);
  XBEE_SER.write((byte) 0);
}


