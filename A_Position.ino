unsigned int posCount = 0;

/************************************************************************
 *  aPos() 
 ************************************************************************/
void aPos() {
  unsigned int posLoop = 0;
  int oldState = false;
  boolean running = false;
  
  int action = BRAKE;
  
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  timeTrigger = micros();
  motorInitPosition();
  setBlink(RED_LED_PIN, BLINK_SF);
  while(mode == MODE_POSITION) { // main loop
    commonTasks();
    while (isDumpingData) {
      setStateBit(TP_STATE_RUN_READY, false);      dumpData();
      delay(50);
    }

    // Do the timed loop
    if (timeMicroseconds > timeTrigger) {
      timeTrigger += 2500; // 400/sec

      boolean newState = isStateBit(TP_STATE_RUN_READY);
      if (newState && !oldState) {
        // A transition to running
        oldState = true;
        running = true;
        digitalWrite(YELLOW_LED_PIN, HIGH);
        posCount = 0;
      }
      else if (!newState && oldState) {
        oldState = false;
        digitalWrite(YELLOW_LED_PIN, LOW);
      }
      
      if (running) {
        if (posCount < tVal) action = FWD;
        else if (posCount < (tVal + uVal)) action = BKWD;
        else {
          action = COAST;
          running = false;
        }
        setMotor(MOTOR_RIGHT, action, 255);
        posCount++;
      }
      if ((++posLoop % 40) == 0) { // 10/sec
        tpDebug = tickPositionRight;
        sendStatusFrame(XBEE_PC);
      }
    } // end timed loop
  } // end while(mode == MODE_POSITION)
}



/************************************************************************
 *  motorInitPos() 
 ************************************************************************/
void motorInitPosition() {
  // Set the pin modes
  pinMode(MOT_RIGHT_PWML, OUTPUT);
  pinMode(MOT_RIGHT_DIR, OUTPUT);
  pinMode(MOT_LEFT_PWML, OUTPUT);
  pinMode(MOT_LEFT_DIR, OUTPUT);

  setMotor(MOTOR_RIGHT, COAST, 0);

  detachInterrupt(MOT_RIGHT_ENCA);
  attachInterrupt(MOT_RIGHT_ENCA, posIsrRight, CHANGE);
}

void posIsrRight() {
  static boolean encAStat;
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsRight++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();  
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
    
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickPositionRight++;
  } 
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickPositionRight--;
  }
  mWsFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
addLog((long) tickTimeRight,
       (short) tickPositionRight,
       (short) posCount,
       (short) mWsFpsRight,
       (short) 0, 
       (short) 0,
       (short) 0);
}

