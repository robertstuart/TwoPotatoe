//#include <DueTimer.h>
//#define TIMER_RIGHT Timer5
//#define TIMER_LEFT Timer8


unsigned long timerPulseEndRight = 0L;
unsigned long timerWaitEndRight = 0L;
unsigned long timerPulseEndLeft = 0L;
unsigned long timerWaitEndLeft = 0L;
int timerStateRight = TIMER_IDLE;
int timerStateLeft = TIMER_IDLE;
int mWsFpsRight = 0;

void motorInitTp() {
  // Set the pin modes
  pinMode(MOT_RIGHT_INA, OUTPUT);
  pinMode(MOT_LEFT_INA, OUTPUT);
  pinMode(MOT_RIGHT_INB, OUTPUT);
  pinMode(MOT_LEFT_INB, OUTPUT);
  //  pinMode(MOT_RIGHT_PWM, OUTPUT);
//  pinMode(MOT_LEFT_PWM, OUTPUT);

  setMotor(MOTOR_RIGHT, COAST, 0);
  setMotor(MOTOR_LEFT, COAST, 0);

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
  mWsFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;
  
  // Compute the target tick position.
//  long time = (long) (micros() - tp5LoopTimeR);
//  long ttdR = loopTickDistanceR + (time  * TICKS_PER_TFOOT * fpsRightLong) / 10000000L; // ticks per period @ 1 fps.
//  long tdError = tickDistanceRight - ttdR;
  
  if (targetDirectionRight == FWD) {
    if (mWsFpsRight < targetMFpsRight) {
      setMotor(MOTOR_RIGHT, FWD, 255);
      timerPulseEndRight = tickTimeRight + MOTOR_PULSE_LENGTH;
      timerStateRight = TIMER_PULSE;
    }
    else if (mWsFpsRight < targetBrakeMFpsRight) {
        setMotor(MOTOR_RIGHT, COAST, 0);
        timerStateRight = TIMER_IDLE;
    }
    else if (mWsFpsRight < targetRevMFpsRight) {
        setMotor(MOTOR_RIGHT, BRAKE, 0);
        timerStateRight = TIMER_IDLE;
    }
    else {
        setMotor(MOTOR_RIGHT, BKWD, 0);
        timerStateRight = TIMER_IDLE;
    }
  }
  else if (targetDirectionRight == BKWD) {
    if (mWsFpsRight > targetMFpsRight) {
      setMotor(MOTOR_RIGHT, BKWD, 255);
      timerPulseEndRight = tickTimeRight + MOTOR_PULSE_LENGTH;
      timerStateRight = TIMER_PULSE;
    }
    else if (mWsFpsRight > targetBrakeMFpsRight) {
        setMotor(MOTOR_RIGHT, COAST, 0);
        timerStateRight = TIMER_IDLE;
    }
    else if (mWsFpsRight > targetRevMFpsRight) {
        setMotor(MOTOR_RIGHT, BRAKE, 0);
        timerStateRight = TIMER_IDLE;
    }
    else {
        setMotor(MOTOR_RIGHT, FWD, 0);
        timerStateRight = TIMER_IDLE;
    }
  }
  else { // STOP
    setMotor(MOTOR_RIGHT, BRAKE, 0);
    timerStateRight = TIMER_IDLE;
  }
} // encoderIsrRight()



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
  int mWsFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  if (mode == MODE_PULSE_SEQUENCE) return;
  if (mode == MODE_PWM_SPEED) return;
  
  // Compute the target tick position.
//  long time = (long) (micros() - tp5LoopTimeL);
//  long ttdL = loopTickDistanceL +  (time  * TICKS_PER_TFOOT * fpsLeftLong) / 10000000L; // ticks per period @ 1 fps.

  if (targetDirectionLeft == FWD) {
    if (mWsFpsLeft < targetMFpsLeft) {
      setMotor(MOTOR_LEFT, FWD, 255);
      timerPulseEndLeft = tickTimeLeft + MOTOR_PULSE_LENGTH;
      timerStateLeft = TIMER_PULSE;
    }
    else if (mWsFpsLeft < targetBrakeMFpsLeft) {
        setMotor(MOTOR_LEFT, COAST, 0);
        timerStateLeft = TIMER_IDLE;
    }
    else if (mWsFpsLeft < targetRevMFpsLeft) {
        setMotor(MOTOR_LEFT, BRAKE, 0);
        timerStateLeft = TIMER_IDLE;
    }
    else {
        setMotor(MOTOR_LEFT, BKWD, 0);
        timerStateLeft = TIMER_IDLE;
    }
  }
  else if (targetDirectionLeft == BKWD) {
    if (mWsFpsLeft > targetMFpsLeft) {
      setMotor(MOTOR_LEFT, BKWD, 255);
      timerPulseEndLeft = tickTimeLeft + MOTOR_PULSE_LENGTH;
      timerStateLeft = TIMER_PULSE;
    }
    else if (mWsFpsLeft > targetBrakeMFpsLeft) {
        setMotor(MOTOR_LEFT, COAST, 0);
        timerStateLeft = TIMER_IDLE;
    }
    else if (mWsFpsLeft > targetRevMFpsLeft) {
        setMotor(MOTOR_LEFT, BRAKE, 0);
        timerStateLeft = TIMER_IDLE;
    }
    else {
        setMotor(MOTOR_LEFT, FWD, 0);
        timerStateLeft = TIMER_IDLE;
    }
  }
  else { // STOP
    setMotor(MOTOR_LEFT, BRAKE, 0);
    timerStateLeft = TIMER_IDLE;
  }
} // end encoderIsrLeft();





/*********************************************************
 *
 * checkMotorXXX()
 *
 *    This methode is polled frequently!
 *
 *********************************************************/
void checkMotorRight() {
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_RIGHT, COAST, 0);
    timerStateRight = TIMER_IDLE;
  }
  else if (targetDirectionRight == STOP) {
    setMotor(MOTOR_RIGHT, BRAKE, 0);
    timerStateRight = TIMER_IDLE;
  }
  else {
    switch (timerStateRight) {
    case TIMER_PULSE: // Pulse in progress
      if (timeMicroseconds > timerPulseEndRight) {
        timerStateRight = TIMER_WAIT;
        timerWaitEndRight = timeMicroseconds + waitPeriodRight;
        setMotor(MOTOR_RIGHT, COAST, 0);
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
          setMotor(MOTOR_RIGHT, FWD, 255);
        }
        else {
          setMotor(MOTOR_RIGHT, BKWD, 255);
        }
      }
      break;
    } // end switch(timerRightState)
  }
}


/************************ checkMotorLeft *************************/
void checkMotorLeft() {
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_LEFT, COAST, 0);
    timerStateLeft = TIMER_IDLE;
  }
  else if (targetDirectionLeft == STOP) {
    setMotor(MOTOR_LEFT, BRAKE, 0);
    timerStateLeft = TIMER_IDLE;
  }
  else {
    switch (timerStateLeft) {
    case TIMER_PULSE:
      if (timeMicroseconds > timerPulseEndLeft) {
        timerStateLeft = TIMER_WAIT;
        timerWaitEndLeft = timeMicroseconds + waitPeriodLeft;
        setMotor(MOTOR_LEFT, COAST, 0);
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
          setMotor(MOTOR_LEFT, FWD, 255);
        }
        else {
          setMotor(MOTOR_LEFT, BKWD, 255);
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
  
  targetTickPeriodRight = (long) (ENC_FACTOR / targetSpeed);
  targetBrakePeriodRight = (long) (ENC_BRAKE_FACTOR / targetSpeed);
  targetMFpsRight = (int) (targetSpeed * 1000.0);//////////////////////////////
  targetBrakeMFpsRight = (int) (targetSpeed * 1020.0);// 2% /////////////////////////
  targetRevMFpsRight = (int) (targetSpeed * 1050.0);// 5% ///////////////////////////
  
  if (targetSpeed > 0.02f) targetDirectionRight = FWD;
  else if (targetSpeed < -0.02f) targetDirectionRight = BKWD;
  else targetDirectionRight = STOP;

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
  
  targetTickPeriodLeft = (long) (ENC_FACTOR / targetSpeed);
  targetBrakePeriodLeft = (long) (ENC_BRAKE_FACTOR / targetSpeed);
  targetMFpsLeft = (int) (targetSpeed * 1000.0);//////////////////////////////
  targetBrakeMFpsLeft = (int) (targetSpeed * 980.0);// 2% /////////////////////////
  targetRevMFpsLeft = (int) (targetSpeed * 950.0);// 5% ///////////////////////////
  
  if (targetSpeed > 0.02f)  targetDirectionLeft = FWD;
  else if (targetSpeed < -0.02f) targetDirectionLeft = BKWD;
  else targetDirectionLeft = STOP;

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
void setMotor(int motor, int state, int pw) {
  int pinInA;
  int pinInB;
  int pinPwm;

  if (motor == MOTOR_RIGHT) {
    pinInA = MOT_RIGHT_INA;
    pinInB = MOT_RIGHT_INB;
    pinPwm = MOT_RIGHT_PWM;
    actionRight = state;
  }
  else {
    pinInA = MOT_LEFT_INB; // Reversed
    pinInB = MOT_LEFT_INA;
    pinPwm = MOT_LEFT_PWM;
    actionLeft = state;
  }
  if ((tpState & TP_STATE_RUNNING) == 0) state = BRAKE;
  
  switch(state) {
  case FWD:
    g_APinDescription[pinInA].pPort -> PIO_SODR = g_APinDescription[pinInA].ulPin; // HIGH
    g_APinDescription[pinInB].pPort -> PIO_CODR = g_APinDescription[pinInB].ulPin; // LOW
    analogWrite(pinPwm, pw);
    // g_APinDescription[pinPwm].pPort -> PIO_SODR = g_APinDescription[pinPwm].ulPin; // HIGH
    break;
  case BKWD:
    g_APinDescription[pinInA].pPort -> PIO_CODR = g_APinDescription[pinInA].ulPin; // LOW
    g_APinDescription[pinInB].pPort -> PIO_SODR = g_APinDescription[pinInB].ulPin; // HIGH
    analogWrite(pinPwm, pw);
    //g_APinDescription[pinPwm].pPort -> PIO_SODR = g_APinDescription[pinPwm].ulPin; // HIGH
    break;
  case COAST:
    g_APinDescription[pinInA].pPort -> PIO_CODR = g_APinDescription[pinInA].ulPin; // LOW
    g_APinDescription[pinInB].pPort -> PIO_CODR = g_APinDescription[pinInB].ulPin; // LOW
    analogWrite(pinPwm, 0);
    // g_APinDescription[pinPwm].pPort -> PIO_CODR = g_APinDescription[pinPwm].ulPin; // LOW
    break;
  case BRAKE:
    g_APinDescription[pinInA].pPort -> PIO_SODR = g_APinDescription[pinInA].ulPin; // HIGH
    g_APinDescription[pinInB].pPort -> PIO_SODR = g_APinDescription[pinInB].ulPin; // HIGH
    analogWrite(pinPwm, 255);
    // g_APinDescription[pinPwm].pPort -> PIO_SODR = g_APinDescription[pinPwm].ulPin; // HIGH
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



