// Start delimiter, count MSB, count LSB, API id, Frame id, dest msb, dest lsb, options
byte transmitBuffer[120] = {0x7E, 0, 0, 0x01, 0, 0xFF, 0xFF, 0x04}; 
int transmitBufferLength = 0;
int transmitBufferPtr = 0;
unsigned long transmitNextWriteTime = 0UL;
 
/*********************************************************
 * sendStatusFrame() Send out status data.
 *********************************************************/
void sendStatusFrame(int rfDataLength) { 
  if (isBlockInProgress) return;
  if (++txRateCounter >= txRateDivider) {
    txRateCounter = 0;
    sendArray[TP_SEND_STATE_STATUS] = tpState;
    sendArray[TP_SEND_MODE_STATUS] = mode;
    set2Byte(sendArray, TP_SEND_BATTERY, batteryVolt);
    set2Byte(sendArray, TP_SEND_DEBUG, debugVal);
    sendArray[TP_SEND_MSG_ACK] = ackMsgType;
    set2Byte(sendArray, TP_SEND_MSG_ACKVAL, ackMsgVal);
    
    if (rfDataLength == 0) rfDataLength = TP_SEND_A_VAL;
    sendFrame(rfDataLength);
  }
}
 

/*********************************************************
 *
 * sendFrame()
 *
 *     Send the API frame with the given Data Header
 *     and Data. Prepend the Start Delimiter (0x7E)
 *     and the Length bytes.  Append the Checksum
 *     before sending.
 *
 *********************************************************/
void sendFrame(int dataLength) {
  unsigned int sum = 0;
  // Set the length in bytes 2 & 3.
  transmitBuffer[1] = 0;
  transmitBuffer[2] = dataLength + 5;

  // Copy to a single buffer.
  for (int i = 0; i < dataLength; i++) {
    transmitBuffer[i + 8] = sendArray[i];
  }
  
  // Compute the checksum.
  for (int i = 3; i < (dataLength + 8); i++) {
    sum += transmitBuffer[i];
  }  
  byte checkSum = 0xFF - sum;

  // Start the first byte of the transmit.
  transmitBufferPtr = 0;
  transmitBufferLength = 8 + dataLength;
  transmitBuffer[transmitBufferLength] = checkSum;
  transmitNextWriteTime = 0UL;
  flushSerial();
//  
//  for (int i = 0; i < (transmitBufferLength + 1); i++) {
//    Serial.print(transmitBuffer[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
}


/*********************************************************
 *
 * flushSerial()
 *
 *     Sends another byte out from the output buffer.
 *     Only sends a byte when there has been 
 *     200 microseconds since the last send.
 *     This is necessary since the Due does not have
 *     a transmit buffer.
 *
 *********************************************************/
void flushSerial() {
  if ((transmitBufferPtr <= transmitBufferLength) && (timeMicroseconds > transmitNextWriteTime))  {
    transmitNextWriteTime = timeMicroseconds + 170;
    MYSER.write(transmitBuffer[transmitBufferPtr++]);

  }
}


void set2Byte(byte array[], int index, int value) {
  array[index + 1] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index] = (byte) value;
}
void set4Byte(byte array[], int index, int value) {
  array[index + 3] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index + 2] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index + 1] = (byte) (value & 0xFF);
  value = value >> 8;
  array[index] = (byte) value;
}



/*********************************************************
 *
 * dump()
 *
 *     If CMD_STATE_DUMP & TP_STATE_DUMPING is true, we are
 *     collecting data.  If only TP_STATE_DUMPING is true,
 *     we are dumping it to the PC.
 *     
 *
 *********************************************************/
void dump() {
//  if (    isBitClear(cmdState, CMD_STATE_DUMP) 
//      && (isBitClear(tpState, TP_STATE_DUMPING))) {
//        return;
//  }
//      
//    // Are we are collecting data.
//    if ( isBitSet(cmdState, CMD_STATE_DUMP) 
//        && (isBitSet(tpState, TP_STATE_DUMPING))) {
//      
//  }
}




