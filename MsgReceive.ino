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
  
  tHc = timeMilliseconds;
  msgStr[count] = 0; // Just to be sure.
  
  switch(cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) >0) {
        controllerX = floatVal;
//        if (isHc) hcX = floatVal; 
//        else pcX = floatVal;
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        controllerY = floatVal;
//        if (isHc) hcY = floatVal;
//        else pcY = floatVal;
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
      }
      break;
    case RCV_ROUTE:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isRouteInProgress = (x == 0) ? false : true;
      }
      break;
    case RCV_ROUTE_ES:
      if (sscanf(msgStr, "%d", &x) > 0) {
        isRouteInProgress = (x == 0) ? false : true;
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

