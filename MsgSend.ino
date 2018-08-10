const int RF_DATA_SIZE = 72;
byte rfData[RF_DATA_SIZE];
int rfDataPtr = 0;
//byte txRequestDataFrame[100];
//byte packetBuffer[120] = {0x7E}; 

int dumpPtr, dumpEnd;
int tickDumpPtr, tickDumpEnd;



/******************************************************************************
 * SendStatus??() 
 *****************************************************************************/
void sendStatusBluePc() {
  static int part = 0;
  part = ++part % 4;
  switch (part) {
    case 0:
      sendBMsg(SEND_FPS, 2, wFps);
       break;
    case 1:
      sendBMsg(SEND_SONAR_R, 2, sonarRight);
      break;
    case 2:
      sendBMsg(SEND_SONAR_L, 2, sonarLeft);
      break;
    case 3:
      sendBMsg(SEND_BATT_A, 2, battVolt); 
  }
  sendBMsg(SEND_STATE, getState());
}

void sendStatusXBeeHc() {
  static int part = 0;
  part = ++part % 4;
  switch (part) {
    case 0:
      sendXMsg(SEND_FPS, 2, wFps);
      break;
    case 1:
      sendXMsg(SEND_SONAR_L, 2, sonarLeft);
      sendXMsg(SEND_SONAR_F, 2, sonarFront);
      break;
    case 2:
      sendXMsg(SEND_SONAR_R, 2, sonarRight);
      break;
    case 3:
      sendXMsg(SEND_BATT_A, 2, battVolt);
      break;
    default:
      break;
  }
  sendXMsg(SEND_STATE, getState());

  xTransmitRequest(XBEE_DEST_C1, rfData, rfDataPtr);
  rfDataPtr = 0;
}



int getState() {
  int statusInt = 0;
  if (isRunning)          statusInt += 1;
  if (isRunReady)         statusInt += 2;
  if (isUpright)          statusInt += 4;
  if (isLifted)           statusInt += 8;
  if (isHcActive)         statusInt += 16;
  if (isPcActive)         statusInt += 32;
  if (isRouteInProgress)  statusInt += 64;
  if (isDumpingData)      statusInt += 128;
  if (isHoldHeading)      statusInt += 256;
  if (isSpin)             statusInt += 512;
  if (false)              statusInt += 1024; // empty
  if (isStand)            statusInt += 2048;
  return statusInt;
}


/***********************************************************************.
 * dumpData() Set up to start a data dump.
 ***********************************************************************/
void sendDumpData() {
  dumpEnd = dumpPtr =  dataArrayPtr;
  BLUE_SER.write(SEND_DUMP_DATA);
  BLUE_SER.print(message1);
  BLUE_SER.write((byte) 0);
  BLUE_SER.write(SEND_DUMP_DATA);
  BLUE_SER.print(message2);
  BLUE_SER.write((byte) 0);
  BLUE_SER.write(SEND_DUMP_DATA);
  BLUE_SER.print(message3);
  BLUE_SER.write((byte) 0);
  BLUE_SER.write(SEND_DUMP_DATA);
  BLUE_SER.print(message4);
  BLUE_SER.write((byte) 0);
  isDumpingData = true;
}



/***********************************************************************.
 * dumpTicks() Set up to start a tick dump.
 ***********************************************************************/
void sendDumpTicks() {
  tickDumpEnd = tickDumpPtr =  tickArrayPtr;
  isDumpingTicks = true;
}



/***********************************************************************.
 * dumpData()
 ***********************************************************************/
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
  } else {
    BLUE_SER.print("12345678");
    isDumpingData = false;
  }
  BLUE_SER.write((byte) 0);
}



/*********************************************************
 * dumpTicks()
 *********************************************************/
void dumpTicks() {
  BLUE_SER.write(SEND_DUMP_TICKS);
  tickDumpPtr = (tickDumpPtr + 1) %  TICK_ARRAY_SIZE;
  if (tickDumpPtr != tickDumpEnd) {
    BLUE_SER.print(tArray[tickDumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(uArray[tickDumpPtr]);
  }
  else {
    BLUE_SER.print("12345678");
    isDumpingTicks = false;
  }
  BLUE_SER.write((byte) 0);
}



/*********************************************************
 * send?Msg()
 *********************************************************/
void sendBMsg(int cmd, int precision, double val) {
//  BLUE_SER.write(cmd);
//  BLUE_SER.print(val, precision);
//  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, int val) {
//  BLUE_SER.write(cmd);
//  BLUE_SER.print(val);
//  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, String val) {
//  BLUE_SER.write(cmd);
//  BLUE_SER.print(val);
//  BLUE_SER.write((byte) 0);
}

void sendXMsg(int cmd, int precision, double val) {
  char buf[20];
  int len = sprintf(buf, "%.*f", precision, val);
  xAddMessage(cmd, buf, len);
}

void sendXMsg(int cmd, int val) {
  char buf[10];
  int len = sprintf(buf, "%d", val);
  xAddMessage(cmd, buf, len);
}

void sendXMsg(int cmd, String val) {
  char buf[50];
  int len = val.length();
  if (len >= 50) return;
  val.toCharArray(buf, len + 1);
  xAddMessage(cmd, buf, len);
}

void sendUMsg(int cmd, int precision, double val) {
      UP_SER.write((byte) cmd); 
      UP_SER.print(val, precision); 
      UP_SER.write((byte) 0);
}

void sendUMsg(int cmd, int val) {
      UP_SER.write((byte) cmd); 
      UP_SER.print(val); 
      UP_SER.write((byte) 0);
}

void sendUMsg(int cmd, int precision, String val) {
      UP_SER.write((byte) cmd); 
      UP_SER.print(val); 
      UP_SER.write((byte) 0);
}



/**************************************************************************.
 * xAddMessage()
 *                Add message to rfData
 **************************************************************************/
void xAddMessage(int cmd, char buf[], int len) {
  if ((len + 1 + rfDataPtr) >= RF_DATA_SIZE) return;
  rfData[rfDataPtr++] = cmd;
  for (int i = 0; i < len; i++) {
    rfData[rfDataPtr++] = buf[i];
  }
}



/**************************************************************************.
 * xTransmitRequest()
 *                    Create a Transmit Request data frame from the 
 *                    rfDataFrame and send it out.
 **************************************************************************/
void xTransmitRequest(int dest, byte rfFrame[], int rfLength) { 
  static byte txRequestDataFrame[100];
  static int frameId = 0;
  unsigned int sh, sl;
  frameId = ++frameId % 200;   // ID cycles 1-200
  if (dest == XBEE_DEST_C1) {
    sh = XBEE_C1_SH;
    sl = XBEE_C1_SL;
  } else return;
  txRequestDataFrame[0] = 0x10;  // API identifier value
  txRequestDataFrame[1] = frameId + 1;
  txRequestDataFrame[2] = (sh >> 24) & 0x000000FF;
  txRequestDataFrame[3] = (sh >> 16) & 0x000000FF;
  txRequestDataFrame[4] = (sh >> 8) & 0x000000FF;
  txRequestDataFrame[5] = sh & 0x000000FF;
  txRequestDataFrame[6] = (sl >> 24) & 0x000000FF;
  txRequestDataFrame[7] = (sl >> 16) & 0x000000FF;
  txRequestDataFrame[8] = (sl >> 8) & 0x000000FF;
  txRequestDataFrame[9] = sl & 0x000000FF;
  txRequestDataFrame[10] = 0x24;  // 16-bit network address (PAN ID)
  txRequestDataFrame[11] = 0x56;  // 16-bit network address (PAN ID)
  txRequestDataFrame[12] = 0;     // Raduis
  txRequestDataFrame[13] = 0;     // 0ptions

  for (int i = 0; i < rfLength; i++) {
    txRequestDataFrame[i + 14] = rfFrame[i];
  }
  xTransmitUartFrame(txRequestDataFrame, rfLength + 14);  
}



/**************************************************************************.
 *
 * xTransmitUartFrame()
 *
 *     Set all frame bytes and send out with appropriate
 *     characters escaped.
 **************************************************************************/
void xTransmitUartFrame(byte dataFrame[], int dataFrameLength) {
  static byte preEscPack[100];
  static byte uartXmitFrame[200];
  int sum = 0;

  // Compute the checksum.
  for (int i = 0; i < dataFrameLength; i++) {
    sum += dataFrame[i];
  }  
  byte checkSum = 0xFF - (sum & 0xFF);

  // Fill out preEscPack with the array that must be escaped. That is, minus FE.
  preEscPack[0] = 0;
  preEscPack[1] = dataFrameLength;
  for (int i = 0; i < dataFrameLength; i++) {
    preEscPack[i + 2] = dataFrame[i];
  }
  preEscPack[dataFrameLength + 2] = checkSum;

  // Now put it into a single array with the escaped characters.
  int oPtr = 1;
  uartXmitFrame[0] = 0x7E;
  for (int i = 0; i < (dataFrameLength + 3); i++) {
    int b = preEscPack[i];
    if ((b == 0x7E) || (b == 0x7D) || (b == 0x11) || (b == 0x13)) {
      uartXmitFrame[oPtr++] = 0x7D;
      uartXmitFrame[oPtr++] = b ^ 0x20;
    }
    else {
      uartXmitFrame[oPtr++] = b;
    }
  }
  XBEE_SER.write(uartXmitFrame, oPtr);
}
