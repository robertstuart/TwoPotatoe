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
debugVal = timeMicroseconds - tStart;
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
      } else if (dataPtr <= 100) {
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
  if (packetSource == XBEE_PC) {
    tPc = timeMilliseconds;
  }
  else {
    tHc = timeMilliseconds;
  }
  controllerX = ((float)(get1Byte(TP_RCV_X))) / 128.0f;
  controllerY = ((float)(get1Byte(TP_RCV_Y))) / 128.0f;
  cmdState = packetByteArray[TP_RCV_CMD];  
  mode = packetByteArray[TP_RCV_MODE];
  setValSet(packetByteArray[TP_RCV_VALSET]);
  int msgType = packetByteArray[TP_RCV_MSG_TYPE];
  int msgVal = packetByteArray[TP_RCV_MSG_VAL];
  doMessage(msgType, msgVal);
  
//  switch (mode) {
//  case MODE_PWM_SPEED:
//    if (dataLength >= (TP_RCV_V_VALSET)) { 
//      remotePwR = get2Byte(TP_RCV_T_VALSET);
//      remotePwL = get2Byte(TP_RCV_U_VALSET);
//    }
//    break;
//  case MODE_TP_SPEED:
//    if (dataLength >= (TP_RCV_V_VALSET)) { 
//      remoteTpR = ((float) get2Byte(TP_RCV_T_VALSET)) * 0.01;
//      remoteTpL = ((float) get2Byte(TP_RCV_U_VALSET)) * 0.01;
//    }
//    break;
//  case MODE_TP4:
//    if (dataLength >= (TP_RCV_MAX)) {
//      (*currentValSet).t = ((float) get2Byte(TP_RCV_T_VALSET)) * 0.01;    
//      (*currentValSet).u = ((float) get2Byte(TP_RCV_U_VALSET)) * 0.01;    
//      (*currentValSet).v = ((float) get2Byte(TP_RCV_V_VALSET)) * 0.01;    
//      (*currentValSet).w = ((float) get2Byte(TP_RCV_W_VALSET)) * 0.01;    
//      (*currentValSet).x = ((float) get2Byte(TP_RCV_X_VALSET)) * 0.01;    
//      (*currentValSet).y = ((float) get2Byte(TP_RCV_Y_VALSET)) * 0.01;    
//      (*currentValSet).z = ((float) get2Byte(TP_RCV_Z_VALSET)) * 0.01;    
//    }  
//    break; 
//  }  // End switch(mode)
}

void doMessage(int type, int val) {
  // do something
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
  int value = packetByteArray[offset +1] & 0xFF;
  value += packetByteArray[offset] * 256;
  return (value - 32767);
}


