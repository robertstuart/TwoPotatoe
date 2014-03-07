unsigned int pingTpPCCount = 0;
/***************************************************************
 *
 * cmdXXX()
 *
 *       Send commands to controller
 *
 ***************************************************************/
void cmdSinglePC(byte cmd, boolean b) {
  if (b == true) {
    cmd |= CMD_SINGLE_FLAG;
  }
  Serial.write(cmd);
  Serial.println("");  // Needed somewhere.  TODO fix this
}

void cmdFloatPC(byte cmd, float val) {
  Serial.write(cmd);
  Serial.println(val);
}

void cmdIntPC(byte cmd, int val) {
  Serial.write(cmd);
  Serial.println(val);
}

void cmdLongPC(byte cmd, long val) {
  Serial.write(cmd);
  Serial.println(val);
}


/***************************************************************
 *
 * debugXXX()
 *
 *       Send debug output to the controller
 *
 ***************************************************************/
void debugByte(char msg[], byte val) {
  Serial.write(CMD_DEBUG_MSG);
  Serial.print(msg);
  Serial.println(val, HEX);
}

void debugStr(char msg1[], char msg2[]) {
  Serial.write(CMD_DEBUG_MSG);
  Serial.print("--");
  Serial.print(msg1);
  Serial.println(msg2);
}

void debugFloat(char msg[], float f) {
  Serial.write(CMD_DEBUG_MSG);
  Serial.print("--");
  Serial.print(msg);
  Serial.println(f);
}

void debugInt(char msg[], int i) {
  Serial.write(CMD_DEBUG_MSG);
  Serial.print("--");
  Serial.print(msg);
  Serial.println(i);
}

void debugLong(char msg[], long i) {
  Serial.write(CMD_DEBUG_MSG);
  Serial.print("--");
  Serial.print(msg);
  Serial.println(i);
}

void debugBoolean(char msg[], boolean b) {
    Serial.write(CMD_DEBUG_MSG);
    Serial.print("--");
    Serial.print(msg);
    if (b) {
      Serial.println("true");
    }
    else {
      Serial.println("false");
    }
    Serial.flush();
}




/***************************************************************
 *
 * monitor()
 *
 *       Send values to be monitored by the controller.
 *       These values normally are displayed in set boxes (a-j)
 *       in the controller.
 *
 ***************************************************************/
void monitor() {
  if (aDebug) {
    cmdFloatPC(CMD_A_VAL, aVal);
  }
  if (bDebug) { 
    cmdFloatPC(CMD_B_VAL, bVal);
  }
  if (cDebug) {
    cmdFloatPC(CMD_C_VAL, cVal);
  }
  if (dDebug) { 
    cmdFloatPC(CMD_D_VAL, dVal);
  }
  if (eDebug) { 
    cmdFloatPC(CMD_E_VAL, eVal);
  }
  if (fDebug) { 
    cmdFloatPC(CMD_F_VAL, fVal);
  }
  if (gDebug) { 
    cmdFloatPC(CMD_G_VAL, gVal);
  }
  if (hDebug) { 
    cmdFloatPC(CMD_H_VAL, hVal);
  }
  if (iDebug) { 
    cmdFloatPC(CMD_I_VAL, iVal);
  }
  if (jDebug) { 
    cmdFloatPC(CMD_J_VAL, jVal);
  }
  if (kDebug) { 
    cmdFloatPC(CMD_K_VAL, kVal);
  }
  pingTpPCCount = ++pingTpPCCount % 128;
  cmdSinglePC(CMD_FROM_TP_PING, pingTpPCCount);
  
} // end debug()


void debugOff() {
  aDebug = false;
  bDebug = false;
  cDebug = false;
  dDebug = false;
  eDebug = false;
  fDebug = false;
  gDebug = false;
  hDebug = false;
  iDebug = false;
  jDebug = false;
  kDebug = false;
}



