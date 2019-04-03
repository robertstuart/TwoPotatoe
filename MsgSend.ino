/*****************************************************************************-
 *                        MsgSend
 *****************************************************************************/
const int RF_DATA_SIZE = 72;
byte rfData[RF_DATA_SIZE];
int rfDataPtr = 0;
//byte txRequestDataFrame[100];
//byte packetBuffer[120] = {0x7E}; 

/*****************************************************************************-
 * SendStatus??() 
 *****************************************************************************/
void sendStatusXBeeHc() {
  static int part = 0;
  part++;
  part %= 4;
  switch (part) {
    case 0:
      sendXMsg(SEND_FPS, 2, wFps);
      break;
    case 1:
      break;
    case 2:
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
  if (isHcActive)         statusInt += 16;
  if (false)              statusInt += 1024; // empty
  return statusInt;
}



/*********************************************************
 * send?Msg()
 *********************************************************/
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

void sendUpMsg(int cmd, int precision, float val) {
      UP_SER.write((byte) cmd); 
      UP_SER.print(val, precision); 
      UP_SER.write((byte) 0);
}

void sendUpMsg(int cmd, int val) {
      UP_SER.write((byte) cmd); 
      UP_SER.print(val); 
      UP_SER.write((byte) 0);
}

void sendUpMsg(int cmd, int precision, String val) {
      UP_SER.write((byte) cmd); 
      UP_SER.print(val); 
      UP_SER.write((byte) 0);
}

void sendWaMsg(int cmd, int v1, int v2) {
  WA_SER.write((byte) cmd);
  WA_SER.print(v1);
  WA_SER.print(v2);
  WA_SER.write((byte) 0);
}
  
void sendWaMsg(int cmd, int val) {
      WA_SER.write((byte) cmd); 
      WA_SER.print(val); 
      WA_SER.write((byte) 0);
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
  frameId++;
  frameId %= 200;   // ID cycles 1-200
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

void blink(int color, int pattern) {
  static int blinkGn = BLINK_OFF;
  static int blinkRe = BLINK_OFF;
  static int blinkBu = BLINK_OFF;
  static int blinkYe = BLINK_OFF;

  if ((color == LED_SW_RE) && (blinkRe == pattern)) return;
  if ((color == LED_SW_GN) && (blinkGn == pattern)) return;
  if ((color == LED_SW_BU) && (blinkBu == pattern)) return;
  if ((color == LED_SW_BU) && (blinkYe == pattern)) return;

  // We have a new pattern.  Send it.
  sendWaMsg(SEND_BLINK, LED_SW_YE, pattern);
}
void beep(int tune) {
  
}
