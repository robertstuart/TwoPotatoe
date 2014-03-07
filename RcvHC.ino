const int MAX_PACKET_SIZE = 100;
byte packetBytes[MAX_PACKET_SIZE + 1];
int packetByteCount = 0;
int packetIValue;   // int value
unsigned int packetUIValue;  // unsigned int value
long packetLValue;   // long value
unsigned long packetULValue;   // unsigned long value
int hcControllerX = 0;
int hcControllerY = 0;

void readHController() {
  while (Serial1.available() > 0) {
    byte b = Serial1.read();
    if (b & 0x80) { // command?
      if (b < CMD_PARAM) { // Single-byte command?
        doSingleHCCmd(b);
      }
      else {
        doParamHCCmd(b);
      }
    }
    else {
      if (packetByteCount < MAX_PACKET_SIZE) {
        packetBytes[packetByteCount++] = b;
      }
    }
  } // end while SerialAvailable
}


void doSingleHCCmd(byte cmd) {
  boolean flag =cmd & CMD_SINGLE_FLAG;
  cmd = cmd & (CMD_SINGLE_FLAG ^ 0xFF);  // mask of flag bit

  switch (cmd) {

  default:
    unknownHcSingleCmdErrors++;
    break;
  } // end switch(cmd)
}




void doParamHCCmd(byte cmd) {
  switch(cmd) {
  case CMD_Y:
    if (packet1()) { 
      // Allow pc to change value also.
      if (packetIValue != hcControllerY) {
        hcControllerY = packetIValue;
        controllerY = ((float) packetIValue) / 64.0;
        cmdFloatPC(CMD_Y_STAT, controllerY);  // formward
      }
    }
    break;
  case CMD_X:
    if (packet1()) { 
      // Allow pc to change value also.
      if (packetIValue != hcControllerX) {
        hcControllerX = packetIValue;
        controllerX = ((float) packetIValue) / 64.0;
        cmdFloatPC(CMD_X_STAT, controllerX);  // formward
      }
    }
    break;
  case CMD_RUN_STATE:
    if (packetU1()) {
      runState = packetUIValue;
      algorithmStart();
    }
    break;
  case CMD_VAL_SET:
    if (packetU1()) {
      setValSet(packetUIValue);
    }
    break;
  case CMD_MODE:
    break;
  case CMD_ROTATE:
    if (packet2()) {
      if (!isRotating) {
        gyroZAngle = 0.0f;
        isRotating = true;
        rotateTarget = 0.0f;
      }
      rotateTarget += (float) packetIValue;
    }
    break;
  case CMD_TO_TP_PING:
    if (packetU1()) {
      pingHcTpTime = timeMicroSeconds;
      pingHcTpCount = ++pingHcTpCount % 128;
      if (pingHcTpCount != packetUIValue) {
        pingHcTpErrors++;
        pingHcTpCount = packetUIValue;
      }
    }
    pingReply(pingHcTpCount); // Send all packets to HC.
    break;
  default:
    unknownHcParamCmdErrors++;
    break;
  } // end switch()
  packetByteCount = 0;
}


// Unsigned int, 1 byte
boolean packetU1() {
  if (packetByteCount != 1) {
    byteCountErrorsHc++;
    return false;
  }
  packetUIValue = packetBytes[0];
  return true;
}

//  int, 1 byte
boolean packet1() {
  boolean ret = packetU1();
  packetIValue = ((int) packetUIValue) - 64;
  return ret;
}

// unsigned int, 2 bytes
boolean packetU2() {
  if (packetByteCount != 2) {
    byteCountErrorsHc++;
    return false;
  }
  packetUIValue = packetBytes[0];
  packetUIValue += packetBytes[1] << 7;
  return(true);
}

// int, 2 bytes
boolean packet2() {
  boolean ret = packetU2();
  packetIValue = ((int) packetUIValue) - 8192;
  return(ret);
}

// unsigned int, 4 bytes
boolean packetU4() {
  if (packetByteCount != 4) {
    byteCountErrorsHc++;
    return false;
  }    
  unsigned long b1 = packetBytes[0];
  unsigned long b2 = packetBytes[1];
  unsigned long b3 = packetBytes[2];
  unsigned long b4 = packetBytes[3];
  packetULValue = b1;
  packetULValue += b2 * 128; // left shift operator doesn't work!
  packetULValue += b3 * 128* 128;
  packetULValue += b4 * 128* 128* 128;
  return(true);
}

// int, 4 bytes
boolean packet4() {
  boolean ret = packetU4();
  packetLValue = ((long) packetULValue) - 268435456L;
  return(ret);
}

boolean packetMsg() {
  packetBytes[packetByteCount] = 0;
  return true;
}

