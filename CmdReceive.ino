const int MAX_PACKET_SIZE = 100;

byte packetBytes[MAX_PACKET_SIZE + 1];
int packetByteCount = 0;
int packetValue;   // int value
long packetLValue;   // long value
unsigned long packetULValue;   // unsigned long value
boolean isHcCommand = false;

int hcControllerY;
int hcControllerX;

void readXBee() {
  while (Serial.available() > 0) {
    byte b = Serial.read();
    if (b & 0x80) { // command?
      if (b | CMD_HC_SOURCE) {
        isHcConnected = true;
      }
      else {
        isPcConnected = true;
      }
      b = b & 0x3F;  // mask off cmd bit and source bit
      if (b < CMD_PARAM) { // Single-byte command?
        doSingleCmd(b);
      }
      else {
        doParamCmd(b);
      }
    }
    else {
      if (packetByteCount < MAX_PACKET_SIZE) {
        packetBytes[packetByteCount++] = b;
      }
    }
  } // end while()
}


void doSingleCmd(byte cmd) {
  boolean flag = (cmd & CMD_SINGLE_FLAG) == CMD_SINGLE_FLAG;
  cmd = cmd & 0x0F;  // mask off flag bit

  switch (cmd) {
  case CMD_STREAM:
    isStreaming = flag; 
    break; 
  case CMD_LED:
    digitalWrite(LED_PIN, flag);
    break;
  case CMD_PWR:
    digitalWrite(PWR_PIN, flag);
    debug(flag + 42L);
    break;
  case CMD_RESET: // Reset something.
    //    Wire.begin();
    //    gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
    //    gyro.enableDefault();
    //    gyroXAngle = 0.0F;
    break;
  case CMD_HOME:
    if (flag) {
      home = tickDistanceRight + tickDistanceLeft;
    }
    else {
      home = 0;
    }
    break;
  case CMD_SEQ_LOAD:
    if (mode == MODE_TP_SEQUENCE) {
      readWsSequence();
    }
    else if (mode == MODE_MT_SEQUNCE) {
      readMtSequence();
    }
    //    else if (mode == MODE_PWSEQUENCE) {
    //      readPwSequence();
    //    }
    break;
    
  case CMD_SEQ_START:
    switch (mode) {
      case MODE_TP_SEQUENCE:
        initTpSequence();
        break;
      case MODE_MT_SEQUNCE:
        mtInit();
        break;
      default:
        break;
    }
    break;
  default:
    unknownCmdErrors++;
    break;
  } // end switch(cmd)
}




void doParamCmd(byte cmd) {
  switch(cmd) {
  case CMD_T_SET:
    if (packet2) {
      (*currentValSet).t = packetValue * 0.01;
    }
    break;
  case CMD_U_SET:
    if (packet2) {
      (*currentValSet).u = packetValue * 0.01;
    }
    break;
  case CMD_V_SET:
    if (packet2) {
      (*currentValSet).v = packetValue * 0.01;
    }
    break;
  case CMD_W_SET:
    if (packet2) {
      (*currentValSet).w = packetValue * 0.01;
    }
    break;
  case CMD_X_SET:
    if (packet2) {
      (*currentValSet).x = packetValue * 0.01;
    }
    break;
  case CMD_Y_SET:
    if (packet2) {
      (*currentValSet).y = packetValue * 0.01;
    }
    break;
  case CMD_Z_SET:
    if (packet2) {
      (*currentValSet).z = packetValue * 0.01;
    }
    break;
  case CMD_MODE:
    if (packet1) {
      algorithmInit(packetValue);
    }
    break;
  case CMD_RUN_STATE:
    if (packet1) {
      runState = packetValue;
      algorithmStart();
    }
    break;
  case CMD_LAMP:
    if (packet2()) {
      packetValue = constrain(packetValue, 0, 255);
      analogWrite(REAR_BL_PIN, packetValue);
      lamp = packetValue;
    }  
    break;
  case CMD_Y:
    if (packet1()) { 
      // Allow pc to change value also.
      if (packetValue != hcControllerY) {
        hcControllerY = packetValue;
        controllerY = ((float) packetValue) / 64.0;
//        cmdFloatPC(CMD_Y_STAT, controllerY);  // formward
      }
    }
    break;
  case CMD_X:
    if (packet1()) { 
      // Allow pc to change value also.
      if (packetValue != hcControllerX) {
        hcControllerX = packetValue;
        controllerX = ((float) packetValue) / 64.0;
//        cmdFloatPC(CMD_X_STAT, controllerX);  // formward
      }
    }
    break;
  case CMD_VAL_SET:
    if (packet1()) {
      setValSet(packetValue);
    }
    break;
  case CMD_ROTATE:
    if (packet2()) {
      if (!isRotating) {
        gyroZAngle = 0.0f;
        isRotating = true;
        rotateTarget = 0.0f;
      }
      rotateTarget += (float) packetValue;
    }
    break;
    case CMD_DEBUG_SET:
    if (packet1()) {
      setDebug(packetValue, true);
    }
    break;
    case CMD_DEBUG_UNSET:
    if (packet1()) {
      setDebug(packetValue, false);
    }
 default:
    unknownCmdErrors++;
    break;
  } // end switch()
  packetByteCount = 0;
}

//  int, 1 byte
boolean packet1() {
  if (packetByteCount != 1) {
    byteCountErrorsHc++;
    return false;
  }
  packetValue = packetBytes[0] - 64;
  return true;
}

// int, two 7-bit bytes
boolean packet2() {
  if (packetByteCount != 2) {
    byteCountErrorsHc++;
    return false;
  }
  packetValue = packetBytes[0];
  packetValue += packetBytes[1] << 7;
  packetValue = packetValue - 8192;
  return true;
}


boolean packetMsg() {
  packetBytes[packetByteCount] = 0;
  return true;
}

void readPwSequence() {
  unsigned long sumPw = 0;
  unsigned long sumDur = 0;
  sequenceCount = 0;
  while (true) {

    // Get the pw
    int pw = Serial.parseInt();
    if (pw == 4242) {  // End?
      break;
    }
    pwArray[sequenceCount] = pw ;
    sumPw += pw;

    // Get the duration in centiseconds
    int dur = Serial.parseInt();
    if (dur == 4242) {
      break;
    }
    wsDurArray[sequenceCount++] = dur ;
    sumDur += dur;
  }
//  debugLong("Sum pw: ", sumPw);
//  debugLong("Sum Duration: ", sumDur);
}

// Read the sequence sent as stream to TP.  Put in arrays
void readWsSequence() {
  float sumWs  = 0;
  unsigned long sumDur = 0;
  sequenceCount = 0;
  while (true) {

    // Get the wheel speed
    float ws = Serial.parseFloat();
    if (ws > 10.0) {  // End?
      break;
    }
    wsArray[sequenceCount] = ws ;
    sumWs += ws;

    // Get the duration om centiseconds
    int dur = Serial.parseInt();
    if (dur == 4242) {
      break;
    }
    wsDurArray[sequenceCount++] = dur ;
    sumDur += dur;
  }
//  debugFloat("Sum Wheel Speed: ", sumWs);
//  debugLong("Sum Duration: ", sumDur);
}

// Read MT Sequence sent in stream to TP. Put in arrays.
void readMtSequence() {
  float sumState  = 0;
  unsigned long sumDur = 0;
  sequenceCount = 0;
  while (true) {

    // Get the motor state
    int state = Serial.parseInt();
    mtMotorStateArray[sequenceCount] = state;
    if (state ==  END_MARKER) {  // End?
      break;
    }
    sumState += state;

    // Get the duration in microseconds
    //unsigned long dur = (unsigned long) Serial.parseFloat();
    float x = Serial.parseFloat();
    unsigned long dur = (unsigned long) x;
    mtDurArray[sequenceCount++] = dur ;
    sumDur += dur;
  }
//  debugInt("Sum State: ", sumState);
//  debugLong("Sum Duration: ", sumDur);
}

void setValSet(int newValSet) {
  valSetStat = newValSet;
  if (mode == MODE_TP4) {
    switch (valSetStat) {
    case VAL_SET_A:
      currentValSet = &tp4A;
      break;
    case VAL_SET_B:
      currentValSet = &tp4B;
      break;
    case VAL_SET_C:
      currentValSet = &tp4C;
      break;
    default:  
      currentValSet = &tp4A;
      break;
    }
  }
}

void setDebug(int flagNumber, boolean flag) {
}

