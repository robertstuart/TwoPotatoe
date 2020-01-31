/*****************************************************************************-
                          MsgReceive.ino
 *****************************************************************************/

const int UP_BUFFER_SIZE = 100;
char msgStrUp[UP_BUFFER_SIZE];
int msgStrPtrUp = 0;
int msgCmdUp = 0;
boolean isMessageInProgressUp = false;

const int WA_BUFFER_SIZE = 100;
char msgStrWa[WA_BUFFER_SIZE];
int msgStrPtrWa = 0;
int msgCmdWa = 0;
boolean isMessageInProgressWa = false;

const int X_BUFFER_SIZE = 100;
char msgStrX[X_BUFFER_SIZE];
int msgStrPtrX = 0;
boolean isMessageInProgressX = false;



/*****************************************************************************-
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
    readWa() - Read watchdog processor
 *****************************************************************************/
void readWa() {
  while (WA_SER.available()) {
    byte b = WA_SER.read();
    if (b >= 128) {
      msgStrPtrWa = 0;
      msgCmdWa = b;
      isMessageInProgressWa = true;
    } else {
      if (isMessageInProgressWa) {
        if (msgStrPtrWa >= WA_BUFFER_SIZE) {
          isMessageInProgressWa = false;
        } else if (b == 0) {
          msgStrWa[msgStrPtrWa] = 0;
          doWaMsg(msgCmdWa, msgStrWa, msgStrPtrWa);
          isMessageInProgressWa = false;
        } else {
          msgStrWa[msgStrPtrWa++] = b;
        }
      } else {
        Serial.print("Watchdog message error: "); Serial.println(b);
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
        doHcMsg(cmd, msgVal, msgValPtr, true);
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
    doHcMsg() Act on messages from hand controller
 *****************************************************************************/
void doHcMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal = 0;
  float floatVal = 0.0;
  String ss;
  //  boolean booleanVal = false;
  int button = 0;
  boolean isShift = false;
  boolean isCtrl = false;
  boolean isPress = true;  // switch pressed or released

  msgStr[count] = 0; // Just to be sure.

  switch (cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        controllerX = floatVal;
        sendStatusXBeeHc();
        if (abs(controllerX) < 0.02) controllerX = 0.0;
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        controllerY = floatVal;
        if (abs(controllerY) < 0.02) controllerY = 0.0;
      }
      break;
    case RCV_BUTTON: // Button press on HC
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        button = intVal / 256;
        isShift = (intVal & IS_SHIFT_BIT) != 0;
        isCtrl = (intVal & IS_CTRL_BIT) != 0;
        isPress = (intVal & IS_PRESS_BIT) != 0;
        doHcButton(button, isPress, isShift, isCtrl);
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
  int intVal;
  float floatVal;
  String ss;

  //Serial.println(cmd);
  switch (cmd) {
    case FRUP_STEER: // steer
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        routeSteer = floatVal;
      }
      break;
     case FRUP_FPS: // speed
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        routeFps = floatVal;
      }
      break;
    case FRUP_STAT:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        byte statBits = intVal;
        isRouteInProgress = (statBits & B00000001) > 0;
        isRouteStarted = (statBits & B00000010) > 0;
        if (isRouteInProgress && isRouteStarted) {
          buPattern = BLINK_ON;
          isRunReady = true;
        }
        else if (isRouteInProgress && (!isRouteStarted)) buPattern = BLINK_FF;
        else buPattern = BLINK_OFF;
      }
      if (!isUpRunning) {
          sendWaMsg(SEND_BEEP, T_UP3);
          isUpRunning = true;
          rePattern = BLINK_ON;
      }
      timeUp = timeMilliseconds;
      break;
    case FRUP_MSG:
      sendXMsg(SEND_MESSAGE, msgStr);
      break;
    case FRUP_CAPITCH:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        camPitch = floatVal; //Serial.print(caPitch); Serial.print("   ");
      }
      break;
    default:
      Serial.print("Illegal Up message: "); Serial.println(cmd);
      break;
  }
}



/******************************************************************************
    doWaMsg() - Process message from watchdog processor
 *****************************************************************************/
void doWaMsg(int cmd, char msgStr[], int count) {
  float floatVal;
  switch (cmd) {
    case RCV_BATT:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
//        Serial.println(msgStr);
        battVolt = floatVal;
//        Serial.println(floatVal);
      }
      break;
    case RCV_SWITCH:
      doWaButton((unsigned int) msgStr[0] - '0', msgStr[1] == '1');
      break;

    default:
      Serial.print("Illegal Watchdog message: "); Serial.println("cmd");
      break;
  }
}



/*****************************************************************************-
    doWaButton()  Switch pressed/released on watchdog processor
 *****************************************************************************/
void doWaButton(unsigned int sw, boolean isPressed) {
  const int PWM_INC = 5;
  switch (sw) {
    case LED_SW_GN:
      if ((mode == MODE_PWM) || (mode == MODE_MOTOR_CTRL)) {
        if ((intT + PWM_INC) <= 255) {
          intT += PWM_INC;
          intU = intT;
        }
      }
      break;
    case LED_SW_RE:
      if ((mode == MODE_PWM) || (mode == MODE_MOTOR_CTRL)) {
        if ((intT - PWM_INC) >= -255) {
          intT -= PWM_INC;
          intU = intT;
        }
      }
      break;
    case LED_SW_BU:
      break;
    case LED_SW_YE:
      if (isPressed) isRunReady = !isRunReady;
      break;
    default:
      Serial.print("Illegal switch message.");
  }
}



/*****************************************************************************-
   doHcButton()  Button press on hand controller
*****************************************************************************/
void doHcButton(int button, boolean isPress, boolean isShift, boolean isCtrl) {
  if (!isShift && !isCtrl) { // Neither shift nor control pressed
    switch (button) {
      case BUTTON_1L:   // Run
        isRunReady = true;
        break;
      case BUTTON_1M:   // Stop
        isRunReady = false;
        break;
      case BUTTON_1R:    // Start log
        isLogging = true;
        yePattern = BLINK_ON;
        sendUpMsg(TOUP_START_LOG, true);
       break;
      case BUTTON_2L:   // Route Enable
        sendUpMsg(TOUP_RT_ENABLE, true);
        break;
      case BUTTON_2M: // Get up
        setGetUp();
        break;
      case BUTTON_2R:   // Stop log
        isLogging = false;
        yePattern = BLINK_OFF;
        sendUpMsg(TOUP_START_LOG, false);
       break;
      case BUTTON_3L:   // Route Start
        sendUpMsg(TOUP_RT_START, true);
        break;
      case BUTTON_3M:   // Get down
        setGetDown();
        break;
      case BUTTON_3R:
        break;
      case BUTTON_4L:
        sendUpMsg(TOUP_RT_ENABLE, false);
        break;
      case BUTTON_4M:
        isRunningOnGround = !isRunningOnGround;
        break;
      case BUTTON_4R:
        event();
        break;
      default:
        break;
    }
  } else if (isShift && !isCtrl) { // only shift pressed
    switch (button) {
      case BUTTON_1L:
        isCamPitch = false;
        break;
      case BUTTON_1M:
        break;
      case BUTTON_1R:   // Fast
        speedTuning = 2;
        break;
      case BUTTON_2L:
        isCamPitch = true;
        break;
      case BUTTON_2M:
        break;
      case BUTTON_2R:   // Medium
        speedTuning = 1;
        break;
      case BUTTON_3L:
        break;
      case BUTTON_3M:   // Route +
        sendUpMsg(TOUP_RT_NUM, 1);
        break;
      case BUTTON_3R:  // Slow
        speedTuning = 0;
        break;
      case BUTTON_4L:   // HC off, Reserved
        break;
      case BUTTON_4M:   // Route -
        sendUpMsg(TOUP_RT_NUM, 0);
        break;
      case BUTTON_4R:
        break;
      default:
        break;
    }
  } else if (!isShift && isCtrl) { // only control pressed
    switch (button) {
      case BUTTON_1L:  // V1++
        setV1(true);
        break;
      case BUTTON_1M:  // V2+
        setV2(true);
        break;
      case BUTTON_1R:   // Tune+
        break;
      case BUTTON_2L:   // V1-
        setV1(false);
        break;
      case BUTTON_2M:   // V2-
        setV2(false);
        break;
      case BUTTON_2R:   // Tune -
        break;
      case BUTTON_3L:
        break;
      case BUTTON_3M:
        break;
      case BUTTON_3R:   // Play tune
        break;
      case BUTTON_4L:
        break;
      case BUTTON_4M:
        break;
      case BUTTON_4R:
        break;
      default:
        break;
    }
  }
}

void setV1(bool b) {
  static const float V1_INC = 0.05;  // Set this to the amount to increment/decrement
  static float *v1Addr   = &valZ;    // Set this to point to the variable itself
  if (b) *v1Addr += V1_INC;
  else *v1Addr -= V1_INC;
  sprintf(message, "v1 = %5.2f", *v1Addr);
  sendXMsg(SEND_MESSAGE, message);
  Serial.println(*v1Addr);
}
void setV2(bool b) {
  static const float V2_INC = 0.05;  // Set this to the amount to increment/decrement
  static float *v2Addr   = &valZ;    // Set this to point to the variable itself
  if (b) *v2Addr += V2_INC;
  else *v2Addr -= V2_INC;
  sprintf(message, "v2 = %5.2f", *v2Addr);
  sendXMsg(SEND_MESSAGE, message);
  Serial.println(*v2Addr);
}
void event() {
   // sendUpMsg(TOUP_EVENT, 0); Serial.println("sent event");
   isBowlMode = !isBowlMode;
   if (isBowlMode) sendXMsg(SEND_MESSAGE, "Bowl");
   else sendXMsg(SEND_MESSAGE, "    ");
}
