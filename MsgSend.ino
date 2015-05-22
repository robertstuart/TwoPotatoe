// Start delimiter, count MSB, count LSB, API id, Frame id, dest msb, dest lsb, options
byte transmitBuffer[120] = {0x7E, 0, 0, 0x01, 0, 0xFF, 0xFF, 0x04}; 
byte xbeeBuffer[130] = {0x7E, 0, 0};
int transmitBufferLength = 0;
int transmitBufferPtr = 0;
unsigned long transmitNextWriteTime = 0UL;
int dumpPtr, dumpEnd;

void sendStatusBluePc() {
  sendMsg(SEND_FPS, 2, wheelSpeedFps);
  sendMsg(SEND_PITCH, 1, gaPitch);
  sendMsg(SEND_HEADING, 1, currentMapHeading);
  sendMsg(SEND_SONAR, 2, sonarRight);
  sendMsg(SEND_ROUTE_STEP, routeStepPtr); // integer
  sendStateMessage();
}

void sendStateMessage() {
  int statusInt = 0;
  if (isRunning)    statusInt += 1;
  if (isRunReady)   statusInt += 10;
  if (isUpright)    statusInt += 100;
  if (isOnGround)   statusInt += 1000;
  if (isHcActive)   statusInt += 10000;
  if (isPcActive)   statusInt += 100000;
  if (isRoute)      statusInt += 1000000;
  if (isDumping)    statusInt += 10000000;
  sendMsg(SEND_STATE, statusInt); // integer
}
 
/*********************************************************
 * sendStatusFrame() Send out status data.
 *********************************************************/
void sendStatusFrame(int destId) { 
  static unsigned int mainCycle = 0;
  static unsigned int subCycle = 0;
  int flag, val;
  if (!isDumpingData && !isReceivingBlock) {
    
    mainCycle = ++mainCycle % 3;
    switch (mainCycle) {
      case 0: 
        flag = TP_SEND_FLAG_PITCH;
        val = (float) (gaPitch * 100.0);
        break;
      case 1:
        flag = TP_SEND_FLAG_SPEED;
        val = mWheelSpeedFps / 10;
        break;
      case 2:
        subCycle = ++subCycle % 5;
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
            flag = TP_SEND_FLAG_BATT;
            val = battVolt;
            break;
          case 3:
            flag = TP_SEND_FLAG_VALSET;
            val = vSetStatus;
            break;
          case 4:
            flag = TP_SEND_FLAG_DEBUG;
            val = 997;
            break;
        }
      break;
    }
    sendArray[TP_SEND_FLAG] = flag;
    set2Byte(sendArray, TP_SEND_VALUE, val);
    set2Byte(sendArray, TP_SEND_SONAR, (int) (sonarRight * 10.0));
    set2Byte(sendArray, TP_SEND_HEADING, (int) magHeading);
//    if (destId == BLUETOOTH) sendBlueFrame();
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
}

/*********************************************************
 * sendBlueFrame
 *********************************************************/
void sendBlueFrame() {
}

/*********************************************************
 * dumpData() Set up to start a data dump.
 *********************************************************/
void sendDumpData() {
  dumpEnd = dumpPtr =  dataArrayPtr;
  isDumpingData = true;
}



/*********************************************************
 * dumpData()
 *********************************************************/
void dumpData() {
  BLUE_SER.write(SEND_DUMP_DATA);
  dumpPtr = (dumpPtr + 1) %  DATA_ARRAY_SIZE;
  if (dumpPtr != dumpEnd) {
    BLUE_SER.print(aArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(bArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(cArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(dArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(eArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(fArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(gArray[dumpPtr]);
  }
  else {
    BLUE_SER.print("12345678");
    isDumpingData = false;
  }
  BLUE_SER.write((byte) 0);
//  
//  
//  for (int k = 0; k < 6; k++) {
//    dumpPtr = (dumpPtr + 1) %  DATA_ARRAY_SIZE;
//    if (dumpPtr != dumpEnd) {
//      set4Byte(sendArray, 1 + (16 * k), aArray[dumpPtr]);
//      set2Byte(sendArray, 5 + (16 * k), bArray[dumpPtr]);
//      set2Byte(sendArray, 7 + (16 * k), cArray[dumpPtr]);
//      set2Byte(sendArray, 9 + (16 * k), dArray[dumpPtr]);
//      set2Byte(sendArray, 11 + (16 * k), eArray[dumpPtr]);
//      set2Byte(sendArray, 13 + (16 * k), fArray[dumpPtr]);
//      set2Byte(sendArray, 15 + (16 * k), gArray[dumpPtr]);
//    } 
//    else { // end of data dump
//      set4Byte(sendArray, 1, 0L);
//      sendFrame(XBEE_PC, 17);
//      isDumpingData = false;
//      return;
//    }
//  }
//  sendFrame(XBEE_PC, 1 + (16 * 6));
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

void sendMsg(int cmd, int precision, double val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val, precision);
  BLUE_SER.write((byte) 0);
}

void sendMsg(int cmd, int val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

void sendMsg(int cmd, String val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}


