
/************************************************************************
 *  aPos() 
 ************************************************************************/
void aPos() {
  const float K_BRAKE = 0.00002;
  const float K_ACCEL_SPEED = 0.00001;
  const float K_ACCEL_DIST = 5.0;
  static unsigned int posLoop = 0;
  int pulseDir = FWD;
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
//      int posTargetPosition = (int) (controllerY * 400.0); // +-400 ticks
//      
//      // Compute best estimate of speed
//      int posSpeed = mWsFpsRight;
//      int dir = (posSpeed >= 0) ? +1 : -1;
//      int posLatSpeed = (ENC_FACTOR_M / (timeMicroseconds - tickTimeRight)) * dir;
//      if (dir == 1)  posSpeed = (posSpeed > posLatSpeed) ? posLatSpeed : posSpeed;
//      else posSpeed = (posSpeed < posLatSpeed) ? posLatSpeed : posSpeed;
//      int posErrorDistance = posTargetPosition - tickPositionRight; // distance error
//      int posBrakeDistance = ((int) (((float) (posSpeed * posSpeed)) * K_BRAKE)) * dir;
//      int pwSpeed =  (int) (((float) (posSpeed * posSpeed)) * K_ACCEL_SPEED);
//      int pwDist = (int) (((float) abs(posErrorDistance)) * K_ACCEL_DIST);
//      int pw =  pwSpeed + pwDist;

      boolean newState = isStateBit(TP_STATE_RUN_READY);
      if (newState) {
        if (!oldState) { // A transition to running?
          oldState = true;
          running = true;
          mWsFpsRight = 0;
          // start it off in the direction of the target
          if (tickPositionRight < 0) {
            action = FWD;
            pulseDir = FWD;
          } else {
            action = BKWD;
            pulseDir = BKWD;
          }
        }
      }
      else {
        oldState = false;
      }
      
      if (running) {
        if (pulseDir = FWD) {
          if (tickPositionRight > 0) {
            action = BKWD;
            if (mWsFpsRight < 0) { // reversed direction?
              action = BRAKE;
              running = false;
            }
          }
        }
        else {
          if (tickPositionRight < 0) {
            action = FWD;
            if (mWsFpsRight > 0) {
              action = BRAKE;
              running = false;
            }
          }
        }
      }

//      if (isStateBitSet(TP_STATE_RUN_READY)) {
//        if (posErrorDistance >= 0) {
//          // Target position is in front of tp
//          if (posBrakeDistance < posErrorDistance) action = FWD; // Path A
//          else action = BKWD; // Approaching to fast, reverse.      Path B
//        }
//        else {
//          // Target position is behind tp.
//          if (posBrakeDistance > posErrorDistance) action = BKWD;  // Path C
//          else action = FWD;                                     // Path D
//        }
//        setMotor(MOTOR_RIGHT, action, pw);
//      } // end  if (isStateBitSet(TP_STATE_RUN_READY))
//      else {
//        setMotor(MOTOR_RIGHT, BRAKE, 0);        
//      }

     setMotor(MOTOR_RIGHT, action, 255);
          
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
}

