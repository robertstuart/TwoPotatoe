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
int dataLength = 0;

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
  while (MYSER.available() > 0) {
    byte b = MYSER.read();
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


int doMs(int b) {
  if (packetByteCount++ < 2) {
    return PACKET_MS;
  }
  return PACKET_DELIM;
}

int doTxs(int b) {
  switch (packetByteCount++) {
  case 0:
    return PACKET_TXS;
  case 1:
    //    pcAck = (b == 0) ? true : false;
    // debugInt("pcAck: ", b);
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
    dataLength = packetLength - 5;  // Subtract out non-data bytes.
    dataPtr = 0;
    break;
    // Data or checksum after this point.
  default:
    if (dataPtr == dataLength)	{ // Checksum?
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
  if (packetSource == XBEE_PC) tPc = timeMilliseconds;
  else tHc = timeMilliseconds;
  
  int msgType = packetByteArray[TP_RCV_MSG_TYPE];
  if (msgType < TP_BLOCK_ZERO) {
    if (!isRouteInProgress) {
      controllerX = ((float)(get1Byte(TP_RCV_X))) / 128.0f;
      controllerY = ((float)(get1Byte(TP_RCV_Y))) / 128.0f;
    }
    if (msgType != TP_RCV_MSG_NULL) {
      doMessage(msgType, get2Byte(TP_RCV_MSG_VAL));
    }
  }
  else {
    doRouteBlock();
  }
}



/*********************************************************
 *
 * doMessage()
 *
 *********************************************************/
void doMessage(int type, int val) {
  ackMsgType = type;
  ackMsgVal = val;
  switch (type) {
  case TP_RCV_MSG_MODE:
    mode = val;
    break;
  case TP_RCV_MSG_POWER:
    digitalWrite(PWR_PIN, LOW);
    break;
  case TP_RCV_MSG_STREAM:
    if (val != 0) setStateBit(TP_STATE_DATA, true);
    else setStateBit(TP_STATE_DATA, false);
    break;
  case TP_RCV_MSG_RATE:
    if (val != 0) txRateHL = true;
    else txRateHL = false;;
    break;
  case TP_RCV_MSG_RUN_READY:
    if (val != 0) setStateBit(TP_STATE_RUN_READY, true);
    else setStateBit(TP_STATE_RUN_READY, false);
    break;
  case TP_RCV_MSG_BLOCK: // Starting or ending block of data
    if (val != 0) isBlockInProgress = true;
    else isBlockInProgress = false;
    break;
  case TP_RCV_MSG_ROUTE: // Starting route
    if (val != 0) {
      isRouteInProgress = true;
      routeActionPtr = 0;
      setNewRouteAction();
    }
    else {
      isRouteInProgress = false;
      controllerY = 0.0;
    }
    break;
  case TP_RCV_MSG_LIGHTS: // 
    if (val != 0) {
      digitalWrite(RIGHT_HL_PIN, HIGH);
      digitalWrite(LEFT_HL_PIN, HIGH);
      digitalWrite(REAR_BL_PIN, HIGH);
    }
    else {
      digitalWrite(RIGHT_HL_PIN, LOW);
      digitalWrite(LEFT_HL_PIN, LOW);
      digitalWrite(REAR_BL_PIN, LOW);
    } 
    break;
  default:
    break;    
  }
}


/*********************************************************
 *
 * doRouteBlock()
 *
 *********************************************************/
void doRouteBlock() {
  int aVal, bVal;
  isBlockInProgress = true;
  byte c = packetByteArray[1];
//char cc[] = " ";
//cc[0]= c;
//Serial.print(cc); 
//Serial.print(routeActionPtr);
  if (c == 'B') {
    routeActionPtr = 0;
    routeCurrentAction = 0;
  }
  else if (c == 'E') {
    isBlockInProgress = false;
    routeActionSize = routeActionPtr;
    routeActionPtr = 0;
    txRateHL = false;
    txRateDivider = 5;
    txRateCounter = 0;
  }
  else {
    actionArray[routeActionPtr] = c;
    aVal = get2Byte(2);
    bVal = get2Byte(4);
//Serial.print("  ");
//Serial.print(aVal);
//Serial.print("  ");
//Serial.print(bVal);
    aValArray[routeActionPtr] = aVal;
    bValArray[routeActionPtr] = bVal;
    routeActionPtr++;
    routeActionSize++;
  }
//Serial.println();
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

int get1Byte(int offset) {
  return (int) (packetByteArray[offset] - 127);
}
int get2Byte(int offset) {
  unsigned int value = packetByteArray[offset +1] & 0xFF;
  value += packetByteArray[offset] * 256;
  return (value - 32767);
}



