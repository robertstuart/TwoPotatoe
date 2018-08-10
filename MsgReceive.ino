
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



/******************************************************************************
    readUp()
 *****************************************************************************/
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
          isMessageInProgressUp = false;
        } else {
          msgStrUp[msgStrPtrUp++] = b;
        }
      } else {
        Serial.print("Up serial error: "); Serial.println(b);
      }
    }
  }
}



/******************************************************************************
    readBluetooth()
 *****************************************************************************/
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
          isMessageInProgressB = false;
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
/******************************************************************************

   readXBee()

       Read bytes from the XBee radio, and call
       interpretDataFrame() whenever there is a complete
       data packet.

 *****************************************************************************/
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



/******************************************************************************
   interpretRcvDataFrame()
 *****************************************************************************/
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



/******************************************************************************
    doRFData()
 *****************************************************************************/
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



/******************************************************************************
    doMsg() Act on messages from hand controller
 *****************************************************************************/
void doMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal;
  float floatVal;
  boolean booleanVal;
  String ss;

  msgStr[count] = 0; // Just to be sure.

  switch (cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        controllerX = floatVal;
        sendStatusXBeeHc();
        sendStatusBluePc();
        if (abs(controllerX) < 0.02) controllerX = 0.0;
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        controllerY = floatVal;
        if (abs(controllerY) < 0.02) controllerY = 0.0;
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
        sendUMsg(TOUP_RT_ENABLE, 0);   //  toggle?
        //        isRouteInProgress = !isRouteInProgress;
      }
      break;
    case RCV_RT_START:
      sendUMsg(TOUP_RT_START, 0);
      break;
    case RCV_DUMP_START:
      sendDumpData();
      break;
    case RCV_DUMP_TICKS:
      sendDumpTicks();
      break;
    case RCV_RT_SET:      // 0 to decrease, 1 to increase
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        sendUMsg(TOUP_RT_NUM, intVal);  // Increment route number.
      }
      break;
    case RCV_V1: // Set this to point to intended variable.
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        float inc = 0.1;
        tp7.z = (intVal == 0) ? tp7.z - inc : tp7.z + inc;
      }
      break;
    case RCV_V2: // Set this to point to intended variable.
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        float inc = 0.1;
        tp7.z = (intVal == 0) ? tp7.z - inc : tp7.z + inc;
      }
      break;
    case RCV_LIFT: 
      isLiftDisabled = !isLiftDisabled;
      break;
    case RCV_KILLTP:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        if (intVal == 0) sendUMsg(TOUP_KILLTP, 0);
      }
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}

/******************************************************************************
    doUpMsg()
 *****************************************************************************/
void doUpMsg(int cmd, char msgStr[], int count) {
  static float locX = 0.0;
  int intVal;
  float floatVal;
  boolean booleanVal;
  String ss;

  //Serial.println(cmd);
  switch (cmd) {
    case FRUP_QUERY:
      break;
    case FRUP_SET_LOC_X:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        locX = floatVal; // Set aside so we can do both a once
      }
      break;
    case FRUP_SET_LOC_Y:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        currentLoc.x = (double) locX;
        currentLoc.y = (double) floatVal;
      }
      break;
    case FRUP_SET_HEAD:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        setHeading(floatVal);
      }
      break;
    case FRUP_FPS_DIFF:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        speedAdjustment = floatVal;
      }
      break;
    case FRUP_FPS:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        routeFps = floatVal;
      }
      break;
    case FRUP_RT_NUM:
      if (sscanf(msgStr, "%d", &intVal) > 0) {

      }
      break;
    case FRUP_RUN_READY:
      isRunReady = true;
      break;
    case FRUP_MSG:
      Serial.println(msgStr);
      break;
    case FRUP_STAT:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        isRouteInProgress = (intVal == 0) ? false : true;
        upStatTime = timeMilliseconds;
      }
      break;
    default:
      Serial.print("Illegal Up message: "); Serial.println("cmd");
      break;
  }
}



void setValSet(int newValSet) {
}


