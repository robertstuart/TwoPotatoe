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
  Timer4.attachInterrupt(timer4Isr);
  Timer5.attachInterrupt(timer5Isr);
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
  if (digitalRead(MOT_RIGHT_ENCA) == digitalRead(MOT_RIGHT_ENCB)) {
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

  // Pulse if we are behind. (only fwd at the moment.)
  if (timerStateRight != TIMER_IDLE) {
    return;
  }
  if (targetDirectionRight == FWD) {
    if ((tickPeriodRight > targetTickPeriodRight) || (tickPeriodRight <= 0)) { // Too slow?
      if (targetSpeedRight < MAX_PULSE_SPEED) {
        timerStateRight = TIMER_PULSE;
        Timer4.start(1800);
        //        Timer4.initialize(tp4Ky);
      }
      setMotor(MOTOR_RIGHT, FWD);
    }
    else if (tickPeriodRight < targetBrakePeriodRight) { // Brake?
      setMotor(MOTOR_RIGHT, BRAKE);
    } 
    else { // between target & brake.  Coast.
      setMotor(MOTOR_RIGHT, COAST);
    }
  } // end FWD
  else if (targetDirectionRight == BKWD) {
    if ((tickPeriodRight < targetTickPeriodRight) || (tickPeriodRight >= 0)) { // Too slow?
      if (abs(targetSpeedRight) < MAX_PULSE_SPEED) {
        timerStateRight = TIMER_PULSE;
        Timer4.start(1800);
        //        Timer4.initialize(tp4Ky);
      }
      setMotor(MOTOR_RIGHT, BKWD);
    }
    else if (tickPeriodRight > targetBrakePeriodRight) { // Brake?
      setMotor(MOTOR_RIGHT, BRAKE);
    }
    else { // coast
      setMotor(MOTOR_RIGHT, COAST);
    }
  } // end BKWD
  else { // STOP
    setMotor(MOTOR_RIGHT, BRAKE);
  }
} // end encoderIsrRight()



/******************* encoderIsrLeft() **************************/
void encoderIsrLeft() {
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  if (digitalRead(MOT_LEFT_ENCA) == digitalRead(MOT_LEFT_ENCB)) {
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

  // Pulse if we are behind. (only fwd at the moment.)
  if (timerStateLeft != TIMER_IDLE) {
    return;
  }
  if (targetDirectionLeft == FWD) {
    if ((tickPeriodLeft > targetTickPeriodLeft) || (tickPeriodLeft <= 0)) { // Too slow?
      if (targetSpeedLeft < MAX_PULSE_SPEED) {
        timerStateLeft = TIMER_PULSE;
        //        Timer5.initialize(tp4Ky);
        Timer5.start(1800);
      }
      setMotor(MOTOR_LEFT, FWD);
    }
    else if (tickPeriodLeft < targetBrakePeriodLeft) { // Brake?
      setMotor(MOTOR_LEFT, BRAKE);
    } 
    else { // between target & brake.  Coast.
      setMotor(MOTOR_LEFT, COAST);
    }
  } // end FWD
  else if (targetDirectionLeft == BKWD) {
    if ((tickPeriodLeft < targetTickPeriodLeft)  || (tickPeriodLeft >= 0)) { // Too slow?
      if (abs(targetSpeedLeft) < MAX_PULSE_SPEED) {
        timerStateLeft = TIMER_PULSE;
        //        Timer5.initialize(tp4Ky);
        Timer5.start(1800);
      }
      setMotor(MOTOR_LEFT, BKWD);
    }
    else if (tickPeriodLeft > targetBrakePeriodLeft) { // Brake?
      setMotor(MOTOR_LEFT, BRAKE);
    }
    else { // coast
      setMotor(MOTOR_LEFT, COAST);
    }
  } // end BKWD
  else { // STOP
    setMotor(MOTOR_LEFT, BRAKE);
  }
} // end encoderIsrLeft();


/*********************************************************
 *
 * timer4Isr()
 *
 *    Stop the pulse or set a wait period for a new pulse.
 *    Also used by Home mode.
 *
 *********************************************************/
void timer4Isr() {
  Timer4.stop();
  if (timerStateRight == TIMER_PULSE) {
    setMotor(MOTOR_RIGHT, COAST);

    if (waitPeriodRight > 0) {
      Timer4.start(waitPeriodRight);
      timerStateRight = TIMER_WAIT;
    }
    else {
      timerStateRight = TIMER_IDLE;
    }
  } 
  else { // end of wait period
    timerStateRight = TIMER_IDLE;
  }
}



/****************** timer5Isr() **************************/
void timer5Isr() {
  Timer5.stop();
  if (timerStateLeft == TIMER_PULSE) {
    setMotor(MOTOR_LEFT, COAST);
    if (waitPeriodLeft > 0) {
      Timer5.start(waitPeriodLeft);
      timerStateLeft = TIMER_WAIT;
    }
    else {
      timerStateLeft = TIMER_IDLE;
    }
  } 
  else { // end of wait period
    timerStateLeft = TIMER_IDLE;
  }
}



/*********************************************************
 *
 * checkMotorRight()
 *
 *    This methode is polled as frequently as possible
 *    from loop().  It checks to see if motor has stopped
 *    and needs a pulse to get started again.
 *
 *********************************************************/
void checkMotorRight() {
  if ((tpState & TP_STATE_RUNNING) == 0) {
    noInterrupts();
    setMotor(MOTOR_RIGHT, COAST);
    interrupts(); 
    return;
  }
  if (timerStateRight != TIMER_IDLE) {
    return;
  }  
  if (micros() > (tickTimeRight + abs(targetTickPeriodRight + targetTickPeriodRight))) {
    if (targetDirectionRight == FWD) {
      if (targetSpeedRight < MAX_PULSE_SPEED) {
        timerStateRight = TIMER_PULSE;
        //        Timer4.initialize(tp4Ky);
        Timer4.start(1800);
      }
      noInterrupts();
      setMotor(MOTOR_RIGHT, FWD);
      interrupts();    
    }
    else if (targetDirectionRight == BKWD) {
      if (abs(targetSpeedRight) < MAX_PULSE_SPEED) {
        timerStateRight = TIMER_PULSE;
        //        Timer4.initialize(tp4Ky);
        Timer4.start(1800);
      }
      noInterrupts();
      setMotor(MOTOR_RIGHT, BKWD);
      interrupts();    
    }
    else { // STOP
      noInterrupts();
      setMotor(MOTOR_RIGHT, BRAKE);
      interrupts();    
    }
  } 
}


/************************ checkMotorLeft *************************/
void checkMotorLeft() {
  if ((tpState & TP_STATE_RUNNING) == 0) {
    noInterrupts();
    setMotor(MOTOR_LEFT, COAST);
    interrupts(); 
    return;
  }
  if (timerStateLeft != TIMER_IDLE) {
    return;
  }  
  if (micros() > (tickTimeLeft + abs(targetTickPeriodLeft + targetTickPeriodLeft))) {
    if (targetDirectionLeft == FWD) {
      if (targetSpeedLeft < MAX_PULSE_SPEED) {
        timerStateLeft = TIMER_PULSE;
        //        Timer5.initialize(tp4Ky);
        Timer5.start(1800);
      }
      noInterrupts();
      setMotor(MOTOR_LEFT, FWD);
      interrupts();    
    }
    else if (targetDirectionLeft == BKWD) {
      if (abs(targetSpeedLeft) < MAX_PULSE_SPEED) {
        timerStateLeft = TIMER_PULSE;
        //        Timer5.initialize(tp4Ky);
        Timer5.start(1800);
      }
      noInterrupts();
      setMotor(MOTOR_LEFT, BKWD);
      interrupts();    
    }
    else { // STOP
      noInterrupts();
      setMotor(MOTOR_LEFT, BRAKE);
      interrupts();    
    }
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
    digitalWrite(pinInA, HIGH);
    digitalWrite(pinInB, LOW);
    digitalWrite(pinPwm, HIGH);
    break;
  case BKWD:
    digitalWrite(pinInA, LOW);
    digitalWrite(pinInB, HIGH);
    digitalWrite(pinPwm, HIGH);
    break;
  case COAST:
    digitalWrite(pinInA, LOW);
    digitalWrite(pinInB, LOW);
    digitalWrite(pinPwm, LOW);
    break;
  case BRAKE:
    digitalWrite(pinInA, HIGH);
    digitalWrite(pinInB, HIGH);
    digitalWrite(pinPwm, HIGH);
    break;
  }
}




