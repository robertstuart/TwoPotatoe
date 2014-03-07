
#define CMD_BUFFER_SIZE 30 //3 bytes/command

char cmdBufferA[CMD_BUFFER_SIZE];
int cmdBufferPtrA;
byte cmdA;

char cmdBufferB[CMD_BUFFER_SIZE];
int cmdBufferPtrB;
byte cmdB;

void readPCController() {
  while (Serial.available() > 0) {
    byte b = Serial.read();
    if (cmdA == 0) {  // waiting for command
      if (b & 0x80) { // we have a command
        if (b < CMD_PARAM) { // Single-byte command?
          doPCSingleCmd(b);
        } 
        else {
          cmdA = b;
        }
      } 
      else {
        break; //  TODO This is an error?  Will work on later.
      }
    }
    else { // receiving command parameter
      if ((b == '\n') || (b == '\r')) { // end of command
        cmdBufferA[cmdBufferPtrA] = 0;
        doPCParamCmd(cmdA, cmdBufferA);
        cmdA = 0;
        cmdBufferPtrA = 0;
      }
      else { // another byte for the parameter
        cmdBufferA[cmdBufferPtrA++] = b;
        if (cmdBufferPtrA >= CMD_BUFFER_SIZE) { //error
          cmdA = 0;
          cmdBufferPtrA = 0;
        }
      }
    } // end if else cmd
  } // end while SerialAvailable
}

void doPCSingleCmd(byte cmd) {
  boolean flag =cmd & CMD_SINGLE_FLAG;
  cmd = cmd & (CMD_SINGLE_FLAG ^ 0xFF);  // mask of flag bit

  switch (cmd) {
  case CMD_LED:
    digitalWrite(LED_PIN, flag);
    break;
  case CMD_PWR:
    digitalWrite(PWR_PIN, flag);
    break;
  case CMD_A_DBG:
    aDebug = flag;
    break;
  case CMD_B_DBG:
    bDebug = flag;
    break;
  case CMD_C_DBG:
    cDebug = flag;
    break;
  case CMD_D_DBG:
    dDebug = flag;
    break;
  case CMD_E_DBG:
    eDebug = flag;
    break;
  case CMD_F_DBG:
    fDebug = flag;
    break;
  case CMD_G_DBG:
    gDebug = flag;
    break;
  case CMD_H_DBG:
    hDebug = flag;
    break;
  case CMD_I_DBG:
    iDebug = flag;
    break;
  case CMD_J_DBG:
    jDebug = flag;
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
  case CMD_ALIVE:
    lastHandshake = timeMicroSeconds;
    break;
  case CMD_SEQ_LOAD:
    if (mode == MODE_TP_SEQUENCE) {
      readWsSequence();
    }
    //    else if (mode == MODE_PWSEQUENCE) {
    //      readPwSequence();
    //    }
    break;
  case CMD_SEQ_START:
    initTpSequence();
    initPwmSequence();
    break;
  default:
    unknownPcSingleCmdErrors++;
    break;
  } // end switch(cmd)
}


void doPCParamCmd(byte cmd, char param[]) {
  int pingCount;
  float xx, yy, v,w,x,y,z;
  
  switch (cmd) { 
  case CMD_T_SET:
    (*currentValSet).t = atof(param);
    break;
  case CMD_U_SET:
    (*currentValSet).u = atof(param);
    break;
  case CMD_V_SET:
    (*currentValSet).v = atof(param);
    break;
  case CMD_W_SET:
    (*currentValSet). w = atof(param);
    break;
  case CMD_X_SET:
    (*currentValSet).x = atof(param);
    break;
  case CMD_Y_SET:
    (*currentValSet).y = atof(param);
    break;
  case CMD_Z_SET:
    (*currentValSet).z = atof(param);
    break;
  case CMD_Y:
    if (!hcAlive) { // Ignore if HC is connected.
      controllerY = atof(param);
    }
    break;
  case CMD_X:
    if (!hcAlive) { // Ignore if HC is connected.
      controllerX = atof(param);
    }
    break;
  case CMD_MODE:
    algorithmInit(atoi(param));
    break;
  case CMD_GENERAL:
    general = atof(param);  // This can only come from HC
    break;
  case CMD_RUN_STATE:
    runState = atoi(param);
    algorithmStart();
    break;
  case CMD_VAL_SET: 
    setValSet(atoi(param));
    break;
  case CMD_TO_TP_PING:
    pingCount = atoi(param);
    pingPcTpTime = timeMicroSeconds;
    pingPcTpCount = ++pingPcTpCount % 128;
    if (pingPcTpCount != pingCount) {
      pingPcTpErrors++;
      pingPcTpCount = pingCount;
    }
    break;
  default:
    unknownPcParamCmdErrors++;
    break;
  }
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
    durArray[sequenceCount++] = dur ;
    sumDur += dur;
  }
  debugLong("Sum pw: ", sumPw);
  debugLong("Sum Duration: ", sumDur);
}

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
    durArray[sequenceCount++] = dur ;
    sumDur += dur;
  }
  debugFloat("Sum Wheel Speed: ", sumWs);
  debugLong("Sum Duration: ", sumDur);
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
//
//void setValue(float &val) {
//  debugFloat("V Value: ", *val);
//}
//


