// Start delimiter, count MSB, count LSB, API id, Frame id, dest msb, dest lsb, options
byte transmitBuffer[120] = {0x7E, 0, 0, 0x01, 0, 0xFF, 0xFF, 0x04}; 
byte xbeeBuffer[130] = {0x7E, 0, 0};
int transmitBufferLength = 0;
int transmitBufferPtr = 0;
unsigned long transmitNextWriteTime = 0UL;
int dumpPtr, dumpEnd;

/*********************************************************
 * SendStatus??() 
 *********************************************************/
void sendStatusBluePc() {
  sendBMsg(SEND_FPS, 2, wheelSpeedFps);
  sendBMsg(SEND_PITCH, 1, gaPitch);
  sendBMsg(SEND_HEADING, 1, currentMapHeading);
  sendBMsg(SEND_SONAR, 2, sonarRight);
  sendBMsg(SEND_ROUTE_STEP, routeStepPtr); // integer
  sendBMsg(SEND_BATT, battVolt); // integer
  sendBMsg(SEND_MODE, mode); // integer
  sendBMsg(SEND_VALSET, vSetStatus); // integer 
  sendBMsg(SEND_STATE, getState()); // MUST BE LAST MESSAGE!
}

void sendStatusXBeeHc() {
  sendXMsg(SEND_FPS, 2, wheelSpeedFps);
  sendXMsg(SEND_PITCH, 1, gaPitch);
  sendXMsg(SEND_HEADING, 1, currentMapHeading);
  sendXMsg(SEND_SONAR, 2, sonarRight);
  sendXMsg(SEND_ROUTE_STEP, routeStepPtr); // integer
  sendXMsg(SEND_BATT, battVolt); // integer
  sendXMsg(SEND_MODE, mode); // integer
  sendXMsg(SEND_VALSET, vSetStatus); // integer 
  sendXMsg(SEND_STATE, getState());
}

int getState() {
  int statusInt = 0;
  if (isRunning)          statusInt += 1;
  if (isRunReady)         statusInt += 2;
  if (isUpright)          statusInt += 4;
  if (isOnGround)         statusInt += 8;
  if (isHcActive)         statusInt += 16;
  if (isPcActive)         statusInt += 32;
  if (isRouteInProgress)  statusInt += 64;
  if (isDumpingData)      statusInt += 128;
  if (isHoldHeading)      statusInt += 256;
  if (isHoldFps)          statusInt += 512;
  return statusInt;
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
}



/*********************************************************
 * send?Msg()
 *********************************************************/
void sendBMsg(int cmd, int precision, double val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val, precision);
  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, int val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, String val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

void sendXMsg(int cmd, int precision, double val) {
  XBEE_SER.write(cmd);
  XBEE_SER.print(val, precision);
  XBEE_SER.write((byte) 0);
}

void sendXMsg(int cmd, int val) {
  XBEE_SER.write(cmd);
  XBEE_SER.print(val);
  XBEE_SER.write((byte) 0);
}

void sendXMsg(int cmd, String val) {
  XBEE_SER.write(cmd);
  XBEE_SER.print(val);
  XBEE_SER.write((byte) 0);
}


