const int PACKET_DELIM = 0;
const int PACKET_MSB = 1;
const int PACKET_LSB = 2;
const int PACKET_API_ID = 3;
const int PACKET_RX = 4;
const int PACKET_MS = 5;
const int PACKET_TXS = 6;
int packetInProgress = PACKET_DELIM;
const int PACKETBYTE_MAX = 105;
byte packetByteArray[PACKETBYTE_MAX];
int packetLength = 0;
int rcvLength = 0;
int blockCount = 0;

/*********************************************************
 *
 * readXBee()
 *
 *     Read bytes from the XBee radio, make a data
 *     array of the packet and give it to newPacket().  
 *     If the packet is a TX Status
 *     packet, set the state accordingly.
 *
 *********************************************************/
void readXBee() {
  static boolean escState = false;
  
  while (XBEE_SER.available() > 0) {
    byte b = XBEE_SER.read();
xBeeCount++;    
    // Fix escape sequences
    if (packetInProgress != PACKET_DELIM) {
      if (escState) {
        b = b ^ 0x20;
        escState = false;
      }
      else if (b == 0x7D) {
        escState = true;
        return;
      }
    }
    
    switch(packetInProgress) {
    case PACKET_DELIM:
      if (b == 0x7E) {
        packetByteCount = 0;
        packetInProgress = PACKET_MSB;
      }
      break;
    case PACKET_MSB:
      packetLength = b * 256;
      packetInProgress = PACKET_LSB;
      break;
    case PACKET_LSB:
      packetLength += b;
      packetInProgress = PACKET_API_ID;
      break;
    case PACKET_API_ID:
      switch (b) {
      case 0x81:
        packetInProgress = PACKET_RX;
        break;
      case 0x89:
        packetInProgress = PACKET_TXS;
        break;
      case 0x8A:
        packetInProgress = PACKET_MS;
        break;
      default:
        packetInProgress = PACKET_DELIM;
      }
      break;
    case PACKET_RX:
      packetInProgress = doRx(b);
      break;
    case PACKET_TXS:
      packetInProgress = doTxs(b);
      break;
    case PACKET_MS:
      packetInProgress = doMs(b);
      break;
    } // end switch(packetInProgress)
  } // end while(dataReady)
}  // end readXBee()

// Modem status packet in progress
int doMs(int b) {
  if (packetByteCount++ < 2) {
    return PACKET_MS;
  }
  return PACKET_DELIM;
}

// Transmit status in progress
int doTxs(int b) {
  switch (packetByteCount++) {
  case 0:
    ackFrameNumber = b;
    return PACKET_TXS;
  case 1:
    switch(b) {
    case 0:
      break;
    case 1:
      ackFailure++;
      break;
    case 2:
      ccaFailure++;
      break;
    case 3:
      purgeFailure++;
      break;
    }
    return PACKET_TXS;
  }
  return PACKET_DELIM;
}

// Data packet in progress
int doRx(int b) {
  switch (packetByteCount++) {
  case 0:
    packetSource = b * 256;
    break;
  case 1:
    packetSource += b;
    break;
  case 2:
    signalStrength = b;
    break;
  case 3: // Options
    rcvLength = packetLength - 5;  // Subtract out non-data bytes.
    dataPtr = 0;
    break;
    // Data or checksum after this point.
  default:
    if (dataPtr == rcvLength)	{ // Checksum?
      newPacket();
      return PACKET_DELIM;  // end of RX packet
    } 
    else if (dataPtr <= 100) {
      packetByteArray[dataPtr++] = (byte) b;
    }
  } 
  return PACKET_RX; // still in RX packet
} // end doRX()



/*********************************************************
 *
 * newPacket()
 *
 *     A new RX packet has been received from the 
 *     Hand Controller or PC.  
 *
 *********************************************************/
void newPacket() { 
 static unsigned int packetCount = 0; 
  if (packetSource == XBEE_PC) tPc = timeMilliseconds;
  else tHc = timeMilliseconds;
  int msgType = packetByteArray[TP_RCV_MSG_TYPE];
  if (msgType < TP_BLOCK_NULL) {
    int x = get1Byte(TP_RCV_X);
    int y = get1Byte(TP_RCV_Y);
    controllerX = ((float) x) / 128.0f;
    controllerY = ((float) y) / 128.0f;
    
//Serial.print(packetCount); Serial.print("\t"); Serial.print(x); Serial.print("\t"); Serial.print(controllerX); Serial.print("\t"); Serial.print(y); Serial.print("\t") ;Serial.println(controllerY);
    ackMsgType = msgType;
    ackMsgVal = get2Byte(TP_RCV_MSG_VAL);
    if (msgType != TP_RCV_MSG_NULL) {
      doMessage(msgType, ackMsgVal);
    }
  }
  else if (msgType == TP_BLOCK_PULSE) {
    doPulseBlock();
  } else if (msgType == TP_BLOCK_ROUTE) {
//    doRouteBlock();
  }
}

/*********************************************************
 * doMessage()
 *********************************************************/
void doMessage(int type, int val) {
  int bo;
 
  switch (type) {
  case TP_RCV_MSG_MODE:
    mode = val;
    break;
  case TP_RCV_MSG_VALSET:
    // not implemented
    break;
  case TP_RCV_MSG_BLUE:
    isBluePassthrough = (val == 0) ? false : true;
    break;
  case TP_RCV_MSG_RUN_READY:
    if (val != 0) setStateBit(TP_STATE_RUN_READY, true);
    else setStateBit(TP_STATE_RUN_READY, false);
    tickPositionRight = tickPositionLeft = tickPosition = 0L;
    break;
  case TP_RCV_MSG_LIGHTS: // 
    isLights = true; // don't allow access
    if (val != 0) {
      analogWrite(RIGHT_HL_PIN, 255);
      analogWrite(LEFT_HL_PIN, 255);
      analogWrite(REAR_BL_PIN, 255);
    } else {
      analogWrite(RIGHT_HL_PIN, 0);
      analogWrite(LEFT_HL_PIN, 0);
      analogWrite(REAR_BL_PIN, 0);
    }
    break;
  case TP_RCV_MSG_DSTART:
    sendDumpData();
    break;
  case TP_RCV_MSG_BLOCK:
    isReceivingBlock = true;
    blockCount = 0;
    pulseCount = 0;
    break;
  case TP_RCV_MSG_START_PW:
    oldMode = mode;
    mode = MODE_PULSE_SEQUENCE;
    if (val != 0) isPwData = true;
    break;
  case TP_RCV_MSG_T_VAL:
    tVal = val;
// Serial.println(tVal);
    (*currentValSet).t = ((float) val) / 1000.0;
    break;
  case TP_RCV_MSG_U_VAL:
    uVal = val;
    (*currentValSet).u = ((float) val) / 1000.0;
    break;
  case TP_RCV_MSG_V_VAL:
    vVal = val;
    (*currentValSet).v = ((float) val) / 1000.0;
    break;
  case TP_RCV_MSG_W_VAL:
    wVal = val;
    (*currentValSet).w = ((float) val) / 1000.0;
    break;
  case TP_RCV_MSG_X_VAL:
    xVal = val;
    (*currentValSet).x = ((float) val) / 1000.0;
    break;
  case TP_RCV_MSG_Y_VAL:
    yVal = val;
    (*currentValSet).y = ((float) val) / 100.0;
    break;
  case TP_RCV_MSG_Z_VAL:
    zVal = val;
    (*currentValSet).z = ((float) val) / 1000.0;
    break;
  default:
    break;    
  }
}

/*********************************************************
 *
 * doPulseBlock()
 *
 *********************************************************/
void doPulseBlock() {
  int motor = get1Byte(1);
  int pw = get2Byte(2) * 100;
  if (pw == 0) {
    isReceivingBlock = false;
  }
  else {
    motorArray[pulseCount] = motor;
    pwArray[pulseCount] = pw;
    pulseCount++;
  }
}

/*********************************************************
 *
 * readBluetooth()
 *
 *     Read bytes from the Bluetooth radio, make a data
 *     array of the packet and give it to newPacket().  
 *
 *********************************************************/
void readBluetooth() {
//  static boolean escState = false;
//  static boolean isReceivingBlue = false;
//  static int blueByteCount = 0;
//  static byte blueArray[TP_RCV_MAX];
//  byte b;
//  while (BLUE_SER.available() > 0) {
//    b = BLUE_SER.read();
//    if (isBluePassthrough)  Serial.write(b);
//    else {
////      if (b == 0x7E) Serial.println(); // Debugging
////      if (b < 16) Serial.print(" ");   // Debugging
////      Serial.print(b,HEX);             // Debugging
//      
//      if (b == 0x7E) {
//        blueByteCount = 0;
//        isReceivingBlue = true;
//      }
//      else if (isReceivingBlue) {
//        blueArray[blueByteCount++] = b;
//        if (blueByteCount >= TP_RCV_MAX) {
//          for (int i = 0; i < 5; i++) packetByteArray[i] = blueArray[i];
//          packetByteArray[3] = packetByteArray[3] - 129;
//          packetByteArray[4] = packetByteArray[4] - 129;
//          newPacket();
//          isReceivingBlue = false;
//        }
//      }
//    }
//  }
//
//  // Send character from console to bluetooth while in passthrough mode
//  if (isBluePassthrough) {
//    while (Serial.available() > 0)  {
//      b = Serial.read();
//      BLUE_SER.write(b);
//    }
//  }
}


/*********************************************************
 *
 * doRouteBlock()
 *
 *********************************************************/
//void doRouteBlock() {
//  int aVal, bVal;
//  isBlockInProgress = true;
//  byte c = packetByteArray[1];
////char cc[] = " ";
////cc[0]= c;
////Serial.print(cc); 
////Serial.print(routeActionPtr);
//  if (c == 'B') {
//    routeActionPtr = 0;
//    routeCurrentAction = 0;
//  }
//  else if (c == 'E') {
//    isBlockInProgress = false;
//    routeActionSize = routeActionPtr;
//    routeActionPtr = 0;
//    txRateHL = false;
//    txRateDivider = 5;
//    txRateCounter = 0;
//  }
//  else {
//    actionArray[routeActionPtr] = c;
//    aVal = get2Byte(2);
//    bVal = get2Byte(4);
////Serial.print("  ");
////Serial.print(aVal);
////Serial.print("  ");
////Serial.print(bVal);
//    aValArray[routeActionPtr] = aVal;
//    bValArray[routeActionPtr] = bVal;
//    routeActionPtr++;
//    routeActionSize++;
//  }
////Serial.println();
//}



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

int get1Byte(int offset) {
  return (int) (packetByteArray[offset] - 127);
}
int get2Byte(int offset) {
  unsigned int value = packetByteArray[offset +1] & 0xFF;
  value += packetByteArray[offset] * 256;
  return (value - 32767);
}


