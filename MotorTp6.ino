
void m6MotorInitTp() {
  // Set the pin modes
  pinMode(MOT_RIGHT_INA, OUTPUT);
  pinMode(MOT_LEFT_INA, OUTPUT);
  pinMode(MOT_RIGHT_INB, OUTPUT);
  pinMode(MOT_LEFT_INB, OUTPUT);
  pinMode(MOT_RIGHT_PWM, OUTPUT);
  pinMode(MOT_LEFT_PWM, OUTPUT);

  setMotor(MOTOR_RIGHT, BRAKE);
  setMotor(MOTOR_LEFT, BRAKE);

  attachInterrupt(MOT_RIGHT_ENCA, m6EncoderIsrRight, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, m6EncoderIsrLeft, CHANGE);
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
void m6EncoderIsrRight() {
  unsigned long m6LastTickTime = tickTimeRight;
  tickTimeRight = micros();  
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
    
  if (encA == encB) {
    tickPeriodRight = (long) tickTimeRight - (long) m6LastTickTime;
    tickDistanceRight++;
  } 
  else {
    tickPeriodRight = (long) m6LastTickTime - (long) tickTimeRight;
    tickDistanceRight--;
  }
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_RIGHT, BRAKE);
    return;
  } 
  
  // Compute the target tick position.
  long time = (long) (micros() - tp6LoopTimeR); // time since loop measurement
  long ttdR = loopTickDistanceR + ((time  * TICKS_PER_TFOOT * fpsRightLong) / 10000000L); // should be ttdR ticks at this time
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
void m6EncoderIsrLeft() {
  unsigned long m6LastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  
  if (encA == encB) {
    tickPeriodLeft = (long) m6LastTickTime - (long) tickTimeLeft;
    tickDistanceLeft--;
  } 
  else {
    tickPeriodLeft = (long) tickTimeLeft - (long) m6LastTickTime;
    tickDistanceLeft++;
  }
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_LEFT, COAST);
    return;
  } 
  
  // Compute the target tick position.
  long time = (long) (micros() - tp6LoopTimeL);
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



