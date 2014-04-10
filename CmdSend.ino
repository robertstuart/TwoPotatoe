boolean debugFlag = false;
long debugVal = 0;

// Single-byte command.  No paramater other than flag.
void send0(byte cmd, boolean b) {
  cmd = cmd | 0x80;
  if (b == true) {
    cmd = cmd | CMD_SINGLE_FLAG;
  }
  Serial.write(cmd);
}

// Two-byte command.  Parameter has value between -64 to +63
void send1(byte cmd, int val) {
  cmd = cmd | 0x80;
  val += 64;
  if (val < 0) {
    val = 0;
  }
  else if (val > 0x7F) {
    val = 0x7F;
  }
  byte lb = (byte) val;
  Serial.write(lb);
  Serial.write(cmd);
}

// Three-byte command.  Parameter has value between -8192 and +8192.
void send2(int cmd, unsigned int val) {
  cmd = cmd | 0x80;
  val += 8192;
  if (val < 0) {
    val = 0;
  }
  else if (val > 16383) {
    val = 0x3FFF;
  }
  
  byte lb = ((byte) val) & 0x7F;
  byte hb = (val >> 7) & 0x7F;
  Serial.write(lb);
  Serial.write(hb);
  Serial.write(cmd);
}


void send5(int cmd, long val) {
  cmd = cmd | 0x80;
  val += 268435456L;
  byte b1 = val & 0x7F;
  val = val >> 7;
  byte b2 = val & 0x7F;
  val = val >> 7;
  byte b3 = val & 0x7F;
  val = val >> 7;
  byte b4 = val & 0x7F;
  val = val >> 7;
  byte b5 = val & 0x7F;
  Serial.write(b1);
  Serial.write(b2);
  Serial.write(b3);
  Serial.write(b4);
  Serial.write(b5);
  Serial.write(cmd);
}

void debug(long val) {
  debugFlag = true;
  debugVal = val;
}


/*********************************************************
 *
 * monitor()
 *
 *       Send a "query" with bit indicating if HC is connected.
 *       If "isStreaming", send all monitor data.
 *     
 *
 *********************************************************/
void monitor() {
  if (isStreaming) {
    send5(CMD_A_VAL, streamValueArray[0]);
    send2(CMD_B_VAL, streamValueArray[1]);
    send2(CMD_C_VAL, streamValueArray[2]);
    send2(CMD_D_VAL, streamValueArray[3]);
    send2(CMD_E_VAL, streamValueArray[4]);
    send2(CMD_F_VAL, streamValueArray[5]);
    send2(CMD_G_VAL, streamValueArray[6]);
    send2(CMD_H_VAL, streamValueArray[7]);
    send2(CMD_I_VAL, streamValueArray[8]);
    send2(CMD_J_VAL, streamValueArray[9]);
  } 
  else {
    send1(CMD_RUN_STATE_STATUS, runState);
    send1(CMD_MODE_STATUS, mode);
    send1(CMD_VAL_SET_STAT, valSetStat);
    send2(CMD_BATTVOLT_VAL, (int) (batteryVoltage * 100.0f));
  }
  
  // Always send these out.
  if (debugFlag) {
    send5(CMD_DEBUG, debugVal);;
    debugFlag = false;
  }
  send0(CMD_STREAM_STATUS, isStreaming);
  send0(CMD_QUERY, false);  // Query for commands from controllers.
  
  isHcConnected = false;
  isPcConnected = false;
}

