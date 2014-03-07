unsigned int hcPingInc = 0;

void cmdSingleHC(byte cmd, boolean b) {
  if (b == true) {
    cmd |= CMD_SINGLE_FLAG;
  }
  Serial1.write(cmd);
}

void cmdOneUHC(int cmd, unsigned int val) {
  byte lb = ((byte) val) & 0x7F;
  Serial1.write(lb);
  Serial1.write(cmd);
}

void cmdOneHC(int cmd, int val) {
  cmdOneUHC(cmd, (byte) (val + 64));
}

void cmdTwoUHC(int cmd, unsigned int val) {
  byte lb = ((byte) val) & 0x7F;
  byte hb = (val >> 7) & 0x7F;
  Serial1.write(lb);
  Serial1.write(hb);
  Serial1.write(cmd);
}

void cmdTwoHC(int cmd, int val) {
  cmdTwoUHC(cmd, val + 8192);
}

void cmdFourUHC(int cmd, unsigned long val) {
  byte b1 = val & 0x7F;
  val = val >> 7;
  byte b2 = val & 0x7F;
  val = val >> 7;
  byte b3 = val & 0x7F;
  val = val >> 7;
  byte b4 = val & 0x7F;
  Serial1.write(b1);
  Serial1.write(b2);
  Serial1.write(b3);
  Serial1.write(b4);
  Serial1.write(cmd);
}

void cmdFourHC(int cmd, long val) {
  cmdFourUHC(cmd, val + 268435456L);
}

void cmdMsgHC(char* str) {
  Serial1.print(str);
  Serial1.write(CMD_MSG);
}


// Send ping and all status info to HC.
// This is sent following a ping from the HC to avoid any
// collisions between the two XBee's.
void pingReply(unsigned int count) {
  hcPingInc = ++hcPingInc % 4;
  // Send out a status message before the ping.
  switch(hcPingInc) {
  case 0:
    cmdOneUHC(CMD_RUN_STATE_STAT, runState);
    break;
  case 1:
    cmdOneUHC(CMD_MODE_STAT, mode);
    break;
  case 2:
    cmdOneUHC(CMD_VAL_SET_STAT, valSetStat);
    break;
  case 3:
    cmdTwoUHC(CMD_BATTVOLT_VAL, (int) (batteryVoltage * 100));
    break;
  default:
    break;
  }
  cmdOneUHC(CMD_FROM_TP_PING, count);
}


