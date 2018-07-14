
const int B_BUFFER_SIZE = 100;
char msgStrB[B_BUFFER_SIZE];
int msgStrPtrB = 0;
int msgCmdB = 0;
boolean isMessageInProgressB = false;

const int UP_BUFFER_SIZE = 100;
char msgStrUp[UP_BUFFER_SIZE];
int msgStrPtrUp = 0;
int msgCmdUp = 0;
boolean isMessageInProgressUp = false;

char msgStrX[B_BUFFER_SIZE];
int msgStrPtrX = 0;
boolean isMessageInProgressX = false;


void readUp() {
  while (UP_SER.available()) {
    byte b = UP_SER.read();
    if (b >= 128) {
      msgStrPtrUp = 0;
      msgCmdUp = b;
      isMessageInProgressUp = true;
    } else {
      if (isMessageInProgressUp) {
        if (msgStrPtrUp >= UP_BUFFER_SIZE) {
          isMessageInProgressUp = false;
        } else if (b == 0) {
          msgStrUp[msgStrPtrUp] = 0;
          doUpMsg(msgCmdUp, msgStrUp, msgStrPtrUp);
        } else {
          msgStrUp[msgStrPtrUp++] = b;
        }
      }
    }
  }
}


void   readBluetooth() {
  while (BLUE_SER.available()) {
    byte b = BLUE_SER.read();
    if (b >= 128) {
      msgStrPtrB = 0;
      msgCmdB = b;
      isMessageInProgressB = true;
    } else {
      if (isMessageInProgressB) {
        if (msgStrPtrB >= B_BUFFER_SIZE) {
          isMessageInProgressB = false;
        } else if (b == 0) {
          msgStrB[msgStrPtrB] = 0;
          doMsg(msgCmdB, msgStrB, msgStrPtrB, false);
        } else {
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
  static int cmd;
  int rfPtr = 12;
  char msgVal[100];
  int msgValPtr = 0;
  while (rfPtr < rcvDataFrameLength) {
    byte b = rcvDataFrame[rfPtr];
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
  String ss;
  
  msgStr[count] = 0; // Just to be sure.
 
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
      }
      break;
    case RCV_LIGHTS:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        int k = digitalRead(RIGHT_HL_PIN);
        k = (k == HIGH) ? LOW : HIGH;
        digitalWrite(RIGHT_HL_PIN, k);
        digitalWrite(LEFT_HL_PIN, k);
//        digitalWrite(REAR_TL_PIN, k);
      }
      break;
    case RCV_RT_ENABLE:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        if (intVal == 1) startRoute();
        else stopRoute();
      }
      break;
    case RCV_RT_START:
      isStartReceived = true;
      break;
    case RCV_DUMP_START:
      sendDumpData();
      break;
     case RCV_DUMP_TICKS:
      sendDumpTicks();
      break;
    case RCV_RT_SET:      // 0 to decrease, 1 to increase
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        setRoute((intVal == 0) ? false : true);
      }
      break;
    case SEND_XPOS:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        ;
      }
      break;
    case RCV_V1: // Set this to point to intended variable.
      if(sscanf(msgStr, "%d", &intVal) > 0) {
        float inc = 0.1;
        tp7.z = (intVal == 0) ? tp7.z - inc : tp7.z + inc;
      } 
      break;
    case RCV_V2: // Set this to point to intended variable.
      if(sscanf(msgStr, "%d", &intVal) > 0) {
        float inc = 0.1;
        tp7.z = (intVal == 0) ? tp7.z - inc : tp7.z + inc;
      } 
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}

void doUpMsg(int cmd, char msgStr[], int count) {
  int intVal;
  float floatVal;
  boolean booleanVal;
  String ss;

Serial.print(cmd); Serial.print("\t");
  switch(cmd) {
    case FRUP_QUERY:
      break;
    case FRUP_SET_X:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        Serial.println(floatVal);
      }
      break;
    case FRUP_SET_Y:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        
      }
      break;
    case FRUP_SET_HEAD:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        
      }
      break;
    case FRUP_GO_X:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        
      }
      break;
    case FRUP_GO_Y:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        
      }
      break;
    case FRUP_MSG:
      Serial.println(msgStr);
      break;
    default:
      break;
  }
}



void setValSet(int newValSet) {
}


