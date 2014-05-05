

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
  while (SERIAL.available() > 0) {
    byte b = SERIAL.read();
    if (!isPacketInProgress) {
      if (b == 0x7E) {
        isPacketInProgress = true;
        packetByteCount = 0;
        dataPtr = 0;
        isTxStatusMessage = false;
      }  // Ignore all other bytes.
    } 
    else {
      switch (packetByteCount++) {
      case 0:
        packetLength = b * 256;
        break;
      case 1:
        packetLength += b;
        dataLength = packetLength - 5;
        if (packetLength > (100 + 5)) {
          isPacketInProgress = false; // ERROR!
        }
        break;
      case 2:
        if ((b != 0x81) && (b != 0x89)) {  // must be 81 or 89
          isPacketInProgress = false;          
        }
        if (b == 0x89) {
          isTxStatusMessage = true;
        }
        break;
      case 3:
        if (isTxStatusMessage) {
          txAckFrame = b;
        }
        packetSource = b * 256;
        break;
      case 4:
        if (isTxStatusMessage) {
          if (b == 0) { // success?
            if (txAckFrame >= ACK_PC) { // sent to PC
              msgState = MSG_STATE_PC_ACK;
            }
            else {
              msgState = MSG_STATE_HC_ACK;
            }
          }
          else { // failure.
            if (txAckFrame >= ACK_PC) { // sent to PC
              msgState = MSG_STATE_PC_NAK;
            }
            else {
              msgState = MSG_STATE_HC_NAK;
            }
          }
        }
        packetSource += b;
        break;
      case 5:
        if (isTxStatusMessage) {  // Checksum.
          isPacketInProgress = false;
        }
        packetSignal = b;
        break;
      case 6:
        break;
      default:
        if (dataPtr == dataLength) { // Checksum
          isPacketInProgress = false;
          if (packetSource == XBEE_PC) {
            tPc = timeMilliseconds;
          }
          else {
            tHc = timeMilliseconds;
          }
          newPacket(); // Execute packet
        }
        else {
          if (dataPtr < 100) {
            rcvArray[dataPtr++] = b;
          }
        }
      } // end switch()
    } // end else, processing packet
  } // end while(available)
}



/*********************************************************
 *
 * newPacket()
 *
 *     A new packet has been received from the Hand Controller
 *     or the PC.  The length of the packet will vary depending 
 *     on various states.
 *
 *********************************************************/
void newPacket() {
  
  controllerX = ((float)(get1Byte(TP_RCV_X))) / 128.0f;
  controllerY = ((float)(get1Byte(TP_RCV_Y))) / 128.0f;
  cmdState = rcvArray[TP_RCV_CMD];  
  mode = rcvArray[TP_RCV_MODE];
  setValSet(rcvArray[TP_RCV_VALSET]);

  switch (mode) {
  case MODE_PWM_SPEED:
    if (dataLength >= (TP_RCV_V_VALSET)) { 
      remotePwR = get2Byte(TP_RCV_T_VALSET);
      remotePwL = get2Byte(TP_RCV_U_VALSET);
    }
    break;
  case MODE_TP4:
    if (dataLength >= (TP_RCV_MAX)) {
      (*currentValSet).t = ((float) get2Byte(TP_RCV_T_VALSET)) * 0.01;    
      (*currentValSet).u = ((float) get2Byte(TP_RCV_U_VALSET)) * 0.01;    
      (*currentValSet).v = ((float) get2Byte(TP_RCV_V_VALSET)) * 0.01;    
      (*currentValSet).w = ((float) get2Byte(TP_RCV_W_VALSET)) * 0.01;    
      (*currentValSet).x = ((float) get2Byte(TP_RCV_X_VALSET)) * 0.01;    
      (*currentValSet).y = ((float) get2Byte(TP_RCV_Y_VALSET)) * 0.01;    
      (*currentValSet).z = ((float) get2Byte(TP_RCV_Z_VALSET)) * 0.01;    
      
//      (*currentValSet).t = rcvArray[TP_RCV_T_VALSET] * 0.01;    
//      (*currentValSet).u = rcvArray[TP_RCV_U_VALSET] * 0.01;    
//      (*currentValSet).v = rcvArray[TP_RCV_V_VALSET] * 0.01;    
//      (*currentValSet).w = rcvArray[TP_RCV_W_VALSET] * 0.01;    
//      (*currentValSet).x = rcvArray[TP_RCV_X_VALSET] * 0.01;    
//      (*currentValSet).y = rcvArray[TP_RCV_Y_VALSET] * 0.01;    
//      (*currentValSet).z = rcvArray[TP_RCV_Z_VALSET] * 0.01;  
    }  
    break; 
  }  // End switch(mode)
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
  return (int) (rcvArray[offset] - 127);
}
int get2Byte(int offset) {
  int value = rcvArray[offset +1] & 0xFF;
  value += rcvArray[offset] * 256;
  return (value - 32767);
}


