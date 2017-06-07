
const int B_BUFFER_SIZE = 100;
char msgStrB[B_BUFFER_SIZE];
int msgStrPtrB = 0;
int msgCmdB = 0;
boolean isMessageInProgressB = false;
char msgStrX[B_BUFFER_SIZE];
int msgStrPtrX = 0;
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
          tPc = timeMilliseconds;
        }
      }
    }
  }
}

const int DATA_FRAME_MAX = 72;
byte rcvDataFrame[DATA_FRAME_MAX + 1];
int rcvDataFramePtr = 0;
int rcvPacketCount = 0;
int rcvDataFrameLength = 0;


/*********************************************************
 *
 * readXBee()
 *
 *     Read bytes from the XBee radio, and call
 *     interpretDataFrame() whenever there is a complete 
 *     data packet.
 *
 *********************************************************/
void readXBee() {
  static boolean escState = false;
  
  while (XBEE_SER.available() > 0) {
    byte b = XBEE_SER.read();
    if (b == 0x7e) {
      rcvPacketCount = 0;
      rcvDataFramePtr = 0;
      escState = false;
    } else {
      if (escState) {
        b = b ^ 0x20;
        escState = false;
      }
      else if (b == 0x7D) {
        escState = true;
        return;
      }

      if (rcvPacketCount == 1) rcvDataFrameLength = b * 256;
      else if (rcvPacketCount == 2) rcvDataFrameLength += b;
      else {
        if (rcvDataFramePtr < rcvDataFrameLength) {
          rcvDataFrame[rcvDataFramePtr++] = b;
         } else if (rcvDataFramePtr == rcvDataFrameLength) { // Checksum
          interpretRcvDataFrame();
          rcvDataFramePtr++;  // just in case...
        }
      }
    }
    rcvPacketCount++;
  }
}  // end readXBee()




/**************************************************************************.
 * interpretRcvDataFrame()
 **************************************************************************/
void interpretRcvDataFrame() {
  switch (rcvDataFrame[0]) { // cmdID
    case 0x8A:           // Modem Status
//      Serial.println(rcvDataFrame[4], HEX);
      break;
    case 0x88:           // AT Command Response
      break;
    case 0x97:           // Remote Command Response
      break;
    case 0x8B:           // Transmit Status
//      for (int i = 1; i < 8; i++) {
//        Serial.print(rcvDataFrame[i], HEX); Serial.print(" ");
//      }
//      Serial.println();
      break;
    case 0x90:           // Receive Packet (A0=0)
      doRFData();
      break;
    case 0x91:           // Receive Packet (A)=1)
      break;
    default:
     break;
  }
}

void doRFData() {
//  for (int i = 0; i < 16; i++) {
//    Serial.print(rcvDataFrame[i + 4], HEX); Serial.print(" ");
//  }
//  Serial.println();
  static int cmd;
  int rfPtr = 12;
  char msgVal[100];
  int msgValPtr = 0;
//Serial.print(rcvDataFrameLength); Serial.print("\t");
  while (rfPtr < rcvDataFrameLength) {
    byte b = rcvDataFrame[rfPtr];
//Serial.print(b, HEX); Serial.print(" ");
    if (b < 128) {
      msgVal[msgValPtr++] = b;
    }
    if ((b > 127) || (rfPtr == (rcvDataFrameLength - 1))) {
      if (msgValPtr > 0) {
        doMsg(cmd, msgVal, msgValPtr, true);
        tHc = timeMilliseconds;
      }
      msgValPtr = 0;
      cmd = b;
    }
    rfPtr++;
  }
//  Serial.println();  
}



void doMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal;
  float floatVal;
  boolean booleanVal;
//  int x = 0;
  String ss;
  
  msgStr[count] = 0; // Just to be sure.
//  if ((cmd != RCV_JOYX) && (cmd != RCV_JOYY)) {
//    Serial.print(cmd); Serial.print("  "); Serial.println(msgStr);
//  }
 
  switch(cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) >0) {
        if (isHc) {
          hcX = floatVal;
          if (abs(hcX) > abs(pcX)) controllerX = floatVal;
          sendStatusXBeeHc();
        } else {
          pcX = floatVal;
          if (abs(pcX) > abs(hcX)) controllerX = floatVal;
          sendStatusBluePc();
        }
        if ((abs(hcX) < 0.02) && (abs(pcX) < 0.02)) controllerX = 0.0; 
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        if (isHc) {
          hcY = floatVal;
          if (abs(hcY) > abs(pcY)) controllerY = floatVal;
        } else {
          pcY = floatVal;
          if (abs(pcY) > abs(hcY)) controllerY = floatVal;
        }
        if ((abs(hcY) < 0.02) && (abs(pcY) < 0.02)) controllerY = 0.0;
      }
      break;
    case RCV_RUN:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isRunReady = (intVal != 0);
      }
      break;
    case RCV_MODE:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        mode = intVal;
        Serial.print("Mode: "); Serial.println(mode);
      }
      break;
    case RCV_LIGHTS:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        intVal = (intVal == 0) ? LOW : HIGH;
        digitalWrite(RIGHT_HL_PIN, intVal);
        digitalWrite(LEFT_HL_PIN, intVal);
//        digitalWrite(REAR_TL_PIN, intVal);
      }
      break;
    case RCV_ROUTE:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        if (intVal == 1) startRoute();
        else stopRoute();
      }
      break;
    case RCV_ROUTE_START:
      isStartReceived = true;
      break;
    case RCV_DUMP_START:
      sendDumpData();
      break;
     case RCV_DUMP_TICKS:
      sendDumpTicks();
      break;
    case RCV_HOLD_HEADING:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isHoldHeading = (intVal == 0) ? false : true;
      }
      break;
    case RCV_SPIN:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isSpin = (intVal == 0) ? false : true;
      }
      break;
    case RCV_SET_ROUTE:      // 0 to decrease, 1 to increase
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        setRoute((intVal == 0) ? false : true);
      }
      break;
    case RCV_ROUTE_DATA: 
      loadRouteLine(String(msgStr));
      break;
    case RCV_DELETE_ROUTE:
      break;
    case RCV_STAND:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isStand = (intVal == 0) ? false : true;
        if (isStand) {
          standTPRight = tickPositionRight;
          standTPLeft = tickPositionLeft;
        }
      }
      break;
    case SEND_XPOS:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        Serial.print(floatVal); Serial.print(tab);
      }
      break;
    case SEND_YPOS:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        Serial.println(floatVal);
      }
      break;
    case RCV_T: // valset t
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        if (mode == MODE_2P) (*currentValSet).t = floatVal;
        else tVal = (int) floatVal;
      } 
      break;
    case RCV_U:  // valset u
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        if (mode == MODE_2P) (*currentValSet).u = floatVal;
        else uVal = (int) floatVal;
      } 
      break;
    case RCV_V:  // valset v
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).v = floatVal;
      } 
      break;
    case RCV_W:  // valset w
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).w = floatVal;
      } 
      break;
    case RCV_X:  // valset x
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).x = floatVal;
      } 
      break;
    case RCV_Y:  // valset y
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).y = floatVal;
      } 
      break;
    case RCV_Z:  // valset z
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        (*currentValSet).z = floatVal;
      } 
      break;
    case RCV_RESET_NAV:
//      resetNavigation(0.0);
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}



void setValSet(int newValSet) {
}


