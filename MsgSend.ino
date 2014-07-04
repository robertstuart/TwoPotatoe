// Start delimiter, count MSB, count LSB, API id, Frame id, dest msb, dest lsb, options
byte transmitBuffer[120] = {0x7E, 0, 0, 0x01, 0, 0xFF, 0xFF, 0x04}; 
int transmitBufferLength = 0;
int transmitBufferPtr = 0;
unsigned long transmitNextWriteTime = 0UL;
int dumpPtr, dumpEnd;
 
/*********************************************************
 * sendStatusFrame() Send out status data.
 *********************************************************/
void sendStatusFrame() { 
  if ((timeMilliseconds > statusTrigger) && !isDumpingData && isSerialEmpty) {
    statusTrigger = timeMilliseconds + 50;  // 20/sec
    sendArray[TP_SEND_MODE_STATUS] = mode;
    sendArray[TP_SEND_STATE_STATUS] = tpState;
    set2Byte(sendArray, TP_SEND_BATTERY, batteryVolt);
    set2Byte(sendArray, TP_SEND_DEBUG, debugVal);
    sendFrame(TP_SEND_END);
  }
}
 

/*********************************************************
 *
 * sendFrame()
 *
 *     Set all frame bytes and send out.
 *     This must only be called if the serial buffer is empty.
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

  // Set up transmit buffer for flushSerial()
  transmitBufferPtr = 0;
  transmitBufferLength = 8 + dataLength;
  transmitBuffer[transmitBufferLength] = checkSum;
  transmitNextWriteTime = 0UL;
  isSerialEmpty = false;
}



/*********************************************************
 * sendData() Set up to start a data dump.
 *********************************************************/
void sendData() {
    dumpEnd = dumpPtr =  dataArrayPtr;
    isDumpingData = true;
}



/*********************************************************
 * dumpData()
 *********************************************************/
void dumpData() {
  if (isDumpingData && isSerialEmpty) { 
    sendArray[TP_SEND_MODE_STATUS] = BLOCK_DATA;
    dumpPtr = (dumpPtr + 1) %  DATA_ARRAY_SIZE;
    if (dumpPtr != dumpEnd) {
      set4Byte(sendArray, 1, timeArray[dumpPtr]);
      set4Byte(sendArray, 5, tickArray[dumpPtr]);
      set4Byte(sendArray, 9, angleArray[dumpPtr]);
      set4Byte(sendArray, 13, motorArray[dumpPtr]);
      sendFrame(17);
    } 
    else { // end of data dump
      set4Byte(sendArray, 1, 0L);
      sendFrame(17);
      isDumpingData = false;
    }
  }
}



/*********************************************************
 *
 * flushSerial()
 *
 *     Sends another byte out from the output buffer.
 *     Only sends a byte when there has been 
 *     xxx microseconds since the last send.
 *     This is necessary since the Due does not have
 *     a transmit buffer so a normal send will block until finished.
 *
 *********************************************************/
void flushSerial() {
  if (!isSerialEmpty && (timeMicroseconds > transmitNextWriteTime))  {
    transmitNextWriteTime = timeMicroseconds + 170;
//if (isDumpingData) {
//  Serial.print(transmitBuffer[transmitBufferPtr]);
//  Serial.print(" ");
//  if (transmitBufferPtr == transmitBufferLength) {
//    Serial.println();
//  }
//}
    MYSER.write(transmitBuffer[transmitBufferPtr++]);
    if (transmitBufferPtr > transmitBufferLength) isSerialEmpty = true;
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






