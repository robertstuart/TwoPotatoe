// Start delimiter, count MSB, count LSB, API id, Frame id, dest msb, dest lsb, options
byte transmitBuffer[120] = {0x7E, 0, 0, 0x01, 0, 0xFF, 0xFF, 0x04}; 
byte xbeeBuffer[130] = {0x7E, 0, 0};
int transmitBufferLength = 0;
int transmitBufferPtr = 0;
unsigned long transmitNextWriteTime = 0UL;
int dumpPtr, dumpEnd;
 
/*********************************************************
 * sendStatusFrame() Send out status data.
 *********************************************************/
void sendStatusFrame(int destId) { 
//Serial.println("StatFrame");
  static unsigned int mainCycle = 0;
  static unsigned int subCycle = 0;
  int flag, val;
  if (!isDumpingData && !isReceivingBlock) {
    mainCycle = ++mainCycle % 3;
    switch (mainCycle) {
      case 0: 
        flag = TP_SEND_FLAG_ANGLE;
        val = (float) (gaPitchAngle * 100.0);
        break;
      case 1:
        flag = TP_SEND_FLAG_SPEED;
        val = mWheelSpeedFps / 10;
        break;
      case 2:
        subCycle = ++subCycle % 7;
        switch (subCycle) {
          case 0:
            flag = TP_SEND_FLAG_MODE;
            val = mode;
            break;
          case 1:
            flag = TP_SEND_FLAG_STATE;
            val = tpState;
            break;
          case 2:
            flag = TP_SEND_FLAG_BMBATT;
            val = bmBattVolt;
            break;
          case 3:
            flag = TP_SEND_FLAG_EMBATT;
            val = emBattVolt;
            break;
          case 4:
            flag = TP_SEND_FLAG_LBATT;
            val = lBattVolt;
            break;
          case 5:
            flag = TP_SEND_FLAG_VALSET;
            val = vSetStatus;
            break;
          case 6:
            flag = TP_SEND_FLAG_DEBUG;
            val = tpDebug;
            break;
        }
      break;
    }
    sendArray[TP_SEND_FLAG] = flag;
    set2Byte(sendArray, TP_SEND_VALUE, val);
//        
//    sendArray[TP_SEND_MODE_STATUS] = mode;
//    sendArray[TP_SEND_STATE_STATUS] = tpState;
//    set2Byte(sendArray, TP_SEND_BATTERY, batteryVolt);
//    set2Byte(sendArray, TP_SEND_DEBUG, debugVal);
    sendFrame(destId, TP_SEND_END);
  }
}
 

/*********************************************************
 *
 * sendFrame()
 *
 *     Set all frame bytes and send out.
 *********************************************************/
void sendFrame(int destId, int dataLength) {
  unsigned int sum = 0;
  // Set the length in bytes 2 & 3.
  transmitBuffer[1] = 0;
  transmitBuffer[2] = dataLength + 5;
  transmitBuffer[5] = destId / 256; // MSB
  transmitBuffer[6] = destId & 0xFF; // LSB

  // Copy to a single buffer.
  for (int i = 0; i < dataLength; i++) {
    transmitBuffer[i + 8] = sendArray[i];
  }
  
  // Compute the checksum.
  for (int i = 3; i < (dataLength + 8); i++) {
    sum += transmitBuffer[i];
  }  
  byte checkSum = 0xFF - sum;
  transmitBufferLength = 8 + dataLength;
  transmitBuffer[transmitBufferLength] = checkSum;
  
  // Escape characters
  int oPtr = 1;
  xbeeBuffer[0] = 0x7E;
  for (int i = 1; i < (transmitBufferLength + 1); i++) {
    int b = transmitBuffer[i];
    if ((b == 0x7E) || (b == 0x7D) || (b == 0x11) || (b == 0x13)) {
      xbeeBuffer[oPtr++] = 0x7D;
      xbeeBuffer[oPtr++] = b ^ 0x20;
    }
    else {
      xbeeBuffer[oPtr++] = b;
    }
  }
  
  XBEE_SER.write(xbeeBuffer, oPtr);
//  Serial.print(xbeeBuffer[7]);
//
//  // Set up transmit buffer for flushSerial()
//  transmitBufferPtr = 0;
//  transmitBufferLength = 8 + dataLength;
//  transmitBuffer[transmitBufferLength] = checkSum;
//  transmitNextWriteTime = 0UL;
//  isSerialEmpty = false;
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
    sendArray[TP_SEND_FLAG] = TP_SEND_FLAG_DUMP;
    dumpPtr = (dumpPtr + 1) %  DATA_ARRAY_SIZE;
    if (dumpPtr != dumpEnd) {
      set4Byte(sendArray, 1, aArray[dumpPtr]);
      set4Byte(sendArray, 5, bArray[dumpPtr]);
      set4Byte(sendArray, 9, cArray[dumpPtr]);
      set4Byte(sendArray, 13, dArray[dumpPtr]);
      sendFrame(XBEE_PC, 17);
    } 
    else { // end of data dump
      set4Byte(sendArray, 1, 0L);
      sendFrame(XBEE_PC, 17);
      isDumpingData = false;
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
//void flushSerial() {
//  if (!isSerialEmpty && (timeMicroseconds > transmitNextWriteTime))  {
//    transmitNextWriteTime = timeMicroseconds + 170;
//    MYSER.write(transmitBuffer[transmitBufferPtr++]);
//    if (transmitBufferPtr > transmitBufferLength) isSerialEmpty = true;
//  }
//}


//void set1Byte(byte array[], int offset, int value) {
//  value += 127;
//  if (value < 0) {
//    value = 0;
//  } 
//  else if (value > 255) {
//    value = 255;
//  }
//  array[offset] = (byte) value;		
//}
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






