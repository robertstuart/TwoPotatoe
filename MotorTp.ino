//#include <DueTimer.h>
//#define TIMER_RIGHT Timer5
//#define TIMER_LEFT Timer8


unsigned long timerPulseEndRight = 0L;
unsigned long timerWaitEndRight = 0L;
unsigned long timerPulseEndLeft = 0L;
unsigned long timerWaitEndLeft = 0L;
int timerStateRight = TIMER_IDLE;
int timerStateLeft = TIMER_IDLE;

void motorInitTp() {
  // Set the pin modes
  pinMode(MOT_RIGHT_INA, OUTPUT);
  pinMode(MOT_LEFT_INA, OUTPUT);
  pinMode(MOT_RIGHT_INB, OUTPUT);
  pinMode(MOT_LEFT_INB, OUTPUT);
  pinMode(MOT_RIGHT_PWM, OUTPUT);
  pinMode(MOT_LEFT_PWM, OUTPUT);

  setMotor(MOTOR_RIGHT, COAST);
  setMotor(MOTOR_LEFT, COAST);

  setTargetSpeedRight(0.0);
  setTargetSpeedLeft(0.0);

  attachInterrupt(MOT_RIGHT_ENCA, encoderIsrRight, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeft, CHANGE);
//  TIMER_RIGHT.attachInterrupt(timerAIsr);
//  TIMER_LEFT.attachInterrupt(timerBIsr);
}


/*********************************************************
 *
 * encoderIsrRight()
 *
 *    Responds to interrupts from the encoder.
 *    Controls the motor directly from this isr.
 *
 *********************************************************/
void encoderIsrRight() {
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();  
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
    
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickDistanceRight++;
  } 
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickDistanceRight--;
  }
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_RIGHT, COAST);
    return;
  } 
  
  // Compute the target tick position.
  long time = (long) (micros() - tp5LoopTimeR);
  long ttdR = loopTickDistanceR + (time  * TICKS_PER_TFOOT * fpsRightLong) / 10000000L; // ticks per period @ 1 fps.
  if (targetDirectionRight == FWD) {
    if (tickDistanceRight < ttdR) {
        setMotor(MOTOR_RIGHT, FWD);
        timerPulseEndRight = tickTimeRight + MOTOR_PULSE_LENGTH;
        timerStateRight = TIMER_PULSE;
    }
    else {
        setMotor(MOTOR_RIGHT, BRAKE);
        timerStateRight = TIMER_IDLE;
    }
  }
  else {
    if (tickDistanceRight > ttdR) {
        setMotor(MOTOR_RIGHT, BKWD);
        timerPulseEndRight = tickTimeRight + MOTOR_PULSE_LENGTH;
        timerStateRight = TIMER_PULSE;
    }
    else {
        setMotor(MOTOR_RIGHT, BRAKE);
        timerStateRight = TIMER_IDLE;
    }
  }
} // end encoderIsrRight()



/******************* encoderIsrLeft() **************************/
void encoderIsrLeft() {
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  
  if (encA == encB) {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickDistanceLeft--;
  } 
  else {
    tickPeriodLeft = (long) tickTimeLeft - (long) lastTickTime;
    tickDistanceLeft++;
  }
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_LEFT, COAST);
    return;
  } 
  
  // Compute the target tick position.
  long time = (long) (micros() - tp5LoopTimeL);
  long ttdL = loopTickDistanceL +  (time  * TICKS_PER_TFOOT * fpsLeftLong) / 10000000L; // ticks per period @ 1 fps.
  if (targetDirectionLeft == FWD) {
    if (tickDistanceLeft < ttdL) {
        timerPulseEndLeft = tickTimeLeft + MOTOR_PULSE_LENGTH;
        timerStateLeft = TIMER_PULSE;
        setMotor(MOTOR_LEFT, FWD);
    }
    else {
        setMotor(MOTOR_LEFT, BRAKE);
        timerStateLeft = TIMER_IDLE;
    }
  }
  else {
    if (tickDistanceLeft > ttdL) {
        timerPulseEndLeft = tickTimeLeft + MOTOR_PULSE_LENGTH;
        timerStateLeft = TIMER_PULSE;
        setMotor(MOTOR_LEFT, BKWD);
    }
    else {
        setMotor(MOTOR_LEFT, BRAKE);
        timerStateLeft = TIMER_IDLE;
    }
  }
} // end encoderIsrLeft();



/*********************************************************
 * encRtp6()  
 *********************************************************/
void encRtp6() {
  // Compute the target tick position.
  long time = (long) (micros() - tp5LoopTimeR);
  long ttdR = loopTickDistanceR + (time  * TICKS_PER_TFOOT * fpsRightLong) / 10000000L; // ticks per period @ 1 fps.
  if (targetDirectionRight == FWD) {
    if (tickDistanceRight < ttdR) {
        setMotor(MOTOR_RIGHT, FWD);
        timerPulseEndRight = tickTimeRight + MOTOR_PULSE_LENGTH;
        timerStateRight = TIMER_PULSE;
    }
    else {
        setMotor(MOTOR_RIGHT, BRAKE);
        timerStateRight = TIMER_IDLE;
    }
  }
  else {
    if (tickDistanceRight > ttdR) {
        setMotor(MOTOR_RIGHT, BKWD);
        timerPulseEndRight = tickTimeRight + MOTOR_PULSE_LENGTH;
        timerStateRight = TIMER_PULSE;
    }
    else {
        setMotor(MOTOR_RIGHT, BRAKE);
        timerStateRight = TIMER_IDLE;
    }
  }
}



/*********************************************************
 * encLtp6()  
 *********************************************************/
void encLtp6() {
  // Compute the target tick position.
  long time = (long) (micros() - tp5LoopTimeL);
  long ttdL = loopTickDistanceL +  (time  * TICKS_PER_TFOOT * fpsLeftLong) / 10000000L; // ticks per period @ 1 fps.
  if (targetDirectionLeft == FWD) {
    if (tickDistanceLeft < ttdL) {
        timerPulseEndLeft = tickTimeLeft + MOTOR_PULSE_LENGTH;
        timerStateLeft = TIMER_PULSE;
        setMotor(MOTOR_LEFT, FWD);
    }
    else {
        setMotor(MOTOR_LEFT, BRAKE);
        timerStateLeft = TIMER_IDLE;
    }
  }
  else {
    if (tickDistanceLeft > ttdL) {
        timerPulseEndLeft = tickTimeLeft + MOTOR_PULSE_LENGTH;
        timerStateLeft = TIMER_PULSE;
        setMotor(MOTOR_LEFT, BKWD);
    }
    else {
        setMotor(MOTOR_LEFT, BRAKE);
        timerStateLeft = TIMER_IDLE;
    }
  }
}


/*********************************************************
 *
 * timerAIsr()
 *
 *    Stop the pulse or set a wait period for a new pulse.
 *    Also used by Home mode.
 *
 *********************************************************/
//void timerAIsr() {
//  TIMER_RIGHT.stop();
//  if (timerStateRight == TIMER_PULSE) {
//    setMotor(MOTOR_RIGHT, COAST);
//
//    if (waitPeriodRight > 0) {
//      TIMER_RIGHT.start(waitPeriodRight);
//      timerStateRight = TIMER_WAIT;
//    }
//    else {
//      timerStateRight = TIMER_IDLE;
//    }
//  } 
//  else { // end of wait period
//    timerStateRight = TIMER_IDLE;
//  }
//}



/****************** timerBIsr() **************************/
//void timerBIsr() {
//  TIMER_LEFT.stop();
//  if (timerStateLeft == TIMER_PULSE) {
//    setMotor(MOTOR_LEFT, COAST);
//    if (waitPeriodLeft > 0) {
//      TIMER_LEFT.start(waitPeriodLeft);
//      timerStateLeft = TIMER_WAIT;
//    }
//    else {
//      timerStateLeft = TIMER_IDLE;
//    }
//  } 
//  else { // end of wait period
//    timerStateLeft = TIMER_IDLE;
//  }
//}



/*********************************************************
 *
 * checkMotorXXX()
 *
 *    This methode is polled frequently!
 *
 *********************************************************/
void checkMotorRight() {
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_RIGHT, COAST);
    timerStateRight = TIMER_IDLE;
  }
  else if (targetDirectionRight == STOP) {
    setMotor(MOTOR_RIGHT, BRAKE);
    timerStateRight = TIMER_IDLE;
  }
  else {
    switch (timerStateRight) {
    case TIMER_PULSE: // Pulse in progress
      if (timeMicroseconds > timerPulseEndRight) {
        timerStateRight = TIMER_WAIT;
        timerWaitEndRight = timeMicroseconds + waitPeriodRight;
        setMotor(MOTOR_RIGHT, COAST);
      }
      break;
    case TIMER_WAIT: // Pulse finished, In timout period.
      if (timeMicroseconds > timerWaitEndRight) {
        timerStateRight = TIMER_IDLE;
      }
      break;
    case TIMER_IDLE: // Ready to start a new pulse if necessary.
      if (timeMicroseconds > (tickTimeRight + abs(targetTickPeriodRight * 2))) {
        timerStateRight = TIMER_PULSE;
        timerPulseEndRight = timeMicroseconds + MOTOR_PULSE_LENGTH;
        if (targetDirectionRight == FWD) {
          setMotor(MOTOR_RIGHT, FWD);
        }
        else {
          setMotor(MOTOR_RIGHT, BKWD);
        }
      }
      break;
    } // end switch(timerRightState)
  }
}


/************************ checkMotorLeft *************************/
void checkMotorLeft() {
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_LEFT, COAST);
    timerStateLeft = TIMER_IDLE;
  }
  else if (targetDirectionLeft == STOP) {
    setMotor(MOTOR_LEFT, BRAKE);
    timerStateLeft = TIMER_IDLE;
  }
  else {
    switch (timerStateLeft) {
    case TIMER_PULSE:
      if (timeMicroseconds > timerPulseEndLeft) {
        timerStateLeft = TIMER_WAIT;
        timerWaitEndLeft = timeMicroseconds + waitPeriodLeft;
        setMotor(MOTOR_LEFT, COAST);
      }
      break;
    case TIMER_WAIT:
      if (timeMicroseconds > timerWaitEndLeft) {
        timerStateLeft = TIMER_IDLE;
      }
      break;
    case TIMER_IDLE:
      if (timeMicroseconds > (tickTimeLeft + abs(targetTickPeriodLeft * 2))) {
        timerStateLeft = TIMER_PULSE;
        timerPulseEndLeft = timeMicroseconds + MOTOR_PULSE_LENGTH;
        if (targetDirectionLeft == FWD) {
          setMotor(MOTOR_LEFT, FWD);
        }
        else {
          setMotor(MOTOR_LEFT, BKWD);
        }
      }
      break;
    } // end switch(timerLeftState)
  }
}



/*********************************************************
 *
 * setTargetSpeedRight()
 *
 *    Set the targetTickPeriodRight targetBrakePeriodRight
 *    and the waitPeriodRight given the targetSpeed.
 *
 *********************************************************/
void setTargetSpeedRight(float targetSpeed) {
  targetSpeedRight = targetSpeed;
  if (targetSpeed > 0.02f) {
    targetTickPeriodRight = (long) (ENC_FACTOR / targetSpeed);
    targetBrakePeriodRight = (long) (ENC_BRAKE_FACTOR / targetSpeed);
    targetDirectionRight = FWD;
  }
  else if (targetSpeed < -0.02f) {
    targetTickPeriodRight = (long) (ENC_FACTOR / targetSpeed);
    targetBrakePeriodRight = (long) (ENC_BRAKE_FACTOR / targetSpeed);
    targetDirectionRight = BKWD;
  }
  else {
    targetDirectionRight = STOP;
  }

  // Set timer wait period specifying when a new pulse can start.
  if (abs(targetSpeed) > MAX_PULSE_SPEED) {
    waitPeriodRight = 0;
  }
  else {
    //    waitPeriodRight = ((MAX_PULSE_SPEED - abs(targetSpeed)) / MAX_PULSE_SPEED) * tp4Kx;
    waitPeriodRight = ((MAX_PULSE_SPEED - abs(targetSpeed)) / MAX_PULSE_SPEED) * 8000;
    if (waitPeriodRight < 200) {
      waitPeriodRight = 0;
    }
  }
}


/********************* setTargetSpeedLeft() ********************/
void setTargetSpeedLeft(float targetSpeed) {
  targetSpeedLeft = targetSpeed;
  if (targetSpeed > 0.02f) {
    targetTickPeriodLeft = (long) (ENC_FACTOR / targetSpeed);
    targetBrakePeriodLeft = (long) (ENC_BRAKE_FACTOR / targetSpeed);
    targetDirectionLeft = FWD;
  }
  else if (targetSpeed < -0.02f) {
    targetTickPeriodLeft = (long) (ENC_FACTOR / targetSpeed);
    targetBrakePeriodLeft = (long) (ENC_BRAKE_FACTOR / targetSpeed);
    targetDirectionLeft = BKWD;
  }
  else {
    targetDirectionLeft = STOP;
  }

  // Set timer wait period specifying when a new pulse can start.
  if (abs(targetSpeed) > MAX_PULSE_SPEED) {
    waitPeriodLeft = 0;
  }
  else {
    //    waitPeriodLeft = ((MAX_PULSE_SPEED - abs(targetSpeed)) / MAX_PULSE_SPEED) * tp4Kx;
    waitPeriodLeft = ((MAX_PULSE_SPEED - abs(targetSpeed)) / MAX_PULSE_SPEED) * 8000;
    if (waitPeriodLeft < 200) {
      waitPeriodLeft = 0;
    }
  }
}

/*********************************************************
 *
 * setMotor()
 *
 *    Set the motor to the specified state.
 *
 *********************************************************/
void setMotor(int motor, int state) {
  int pinInA;
  int pinInB;
  int pinPwm;

  if (motor == MOTOR_RIGHT) {
    pinInA = MOT_RIGHT_INA;
    pinInB = MOT_RIGHT_INB;
    pinPwm = MOT_RIGHT_PWM;
  }
  else {
    pinInA = MOT_LEFT_INB; // Reversed
    pinInB = MOT_LEFT_INA;
    pinPwm = MOT_LEFT_PWM;
  }

  switch(state) {
  case FWD:
    g_APinDescription[pinInA].pPort -> PIO_SODR = g_APinDescription[pinInA].ulPin; // HIGH
    g_APinDescription[pinInB].pPort -> PIO_CODR = g_APinDescription[pinInB].ulPin; // LOW
    g_APinDescription[pinPwm].pPort -> PIO_SODR = g_APinDescription[pinPwm].ulPin; // HIGH
    break;
  case BKWD:
    g_APinDescription[pinInA].pPort -> PIO_CODR = g_APinDescription[pinInA].ulPin; // LOW
    g_APinDescription[pinInB].pPort -> PIO_SODR = g_APinDescription[pinInB].ulPin; // HIGH
    g_APinDescription[pinPwm].pPort -> PIO_SODR = g_APinDescription[pinPwm].ulPin; // HIGH
    break;
  case COAST:
    g_APinDescription[pinInA].pPort -> PIO_CODR = g_APinDescription[pinInA].ulPin; // LOW
    g_APinDescription[pinInB].pPort -> PIO_CODR = g_APinDescription[pinInB].ulPin; // LOW
    g_APinDescription[pinPwm].pPort -> PIO_CODR = g_APinDescription[pinPwm].ulPin; // LOW
    break;
  case BRAKE:
    g_APinDescription[pinInA].pPort -> PIO_SODR = g_APinDescription[pinInA].ulPin; // HIGH
    g_APinDescription[pinInB].pPort -> PIO_SODR = g_APinDescription[pinInB].ulPin; // HIGH
    g_APinDescription[pinPwm].pPort -> PIO_SODR = g_APinDescription[pinPwm].ulPin; // HIGH
    break;
  }
}
//// example: digitalWriteDirect(10, HIGH)
//inline void digitalWriteDirect(int pin, boolean val){
//  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
//  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
//}




/*********************************************************
 *
 *  getSpeedXXX()
 *
 *    returns the floating point fps computed from last encoder period.
 *    Returns a computed value if there have been no recent
 *    encoder interrupts.
 *
 *********************************************************/
void readSpeedRight() {
  unsigned long t = tickTimeRight;
  unsigned long currentPeriod = micros() - t;
  if (currentPeriod > abs(tickPeriodRight)) {
    if (tickPeriodRight > 0) {
      fpsRight = (ENC_FACTOR / (float) currentPeriod);
    } 
    else {
      fpsRight = ((ENC_FACTOR * -1.0) / (float) currentPeriod);
    }
  } 
  else {
    fpsRight = (ENC_FACTOR / (float) tickPeriodRight);
  }
}

void readSpeedLeft() {
  unsigned long t = tickTimeLeft;
  unsigned long currentPeriod = micros() - t;
  if (currentPeriod > abs(tickPeriodLeft)) {
    if (tickPeriodLeft > 0) {
      fpsLeft = (ENC_FACTOR / (float) currentPeriod);
    } 
    else {
      fpsLeft = ((ENC_FACTOR * -1.0) / (float) currentPeriod);
    }
  } 
  else {
    fpsLeft =  (ENC_FACTOR / (float) tickPeriodLeft);
  }
}



/*********************************************************
 *
 * readSpeed()
 *
 *    Sets the speed variables for both wheels and 
 *    sets average.
 *
 *********************************************************/
void readSpeed() {
  readSpeedRight();
  readSpeedLeft();
  tickDistance = tickDistanceRight + tickDistanceLeft;
  wheelSpeedFps = (fpsLeft + fpsRight)/2.0f;
}



