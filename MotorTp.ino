void motorInitTp() {
  setTargetSpeedRight(0.0);
  setTargetSpeedLeft(0.0);
  attachInterrupt(0, encoderIsrRight, CHANGE);
  attachInterrupt(1, encoderIsrLeft, CHANGE);
  Timer4.attachInterrupt(timer4Isr);
  Timer5.attachInterrupt(timer5Isr);
  DDRA = B00000111; // Pins 22-24
  DDRC = B00000111; // Pins 37-35
  MOTOR_PORT_RIGHT = MOTOR_COAST;
  MOTOR_PORT_LEFT = MOTOR_COAST;
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
  if (digitalRead(MOT_RIGHT_ENCA) != digitalRead(MOT_RIGHT_ENCB)) {
    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
    tickDistanceRight++;
  } 
  else {
    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
    tickDistanceRight--;
  }
  if (runState != STATE_RUNNING) {
    MOTOR_PORT_RIGHT = MOTOR_COAST;
    return;
  } 
  if (mode == MODE_HOME) {
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
        Timer4.initialize(1800);
//        Timer4.initialize(tp4Ky);
      }
      MOTOR_PORT_RIGHT = MOTOR_FWD;
    }
    else if (tickPeriodRight < targetBrakePeriodRight) { // Brake?
      MOTOR_PORT_RIGHT = MOTOR_BRAKE;
    } 
    else { // between target & brake.  Coast.
      MOTOR_PORT_RIGHT = MOTOR_COAST; 
    }
  } // end FWD
  else if (targetDirectionRight == BKWD) {
    if ((tickPeriodRight < targetTickPeriodRight) || (tickPeriodRight >= 0)) { // Too slow?
      if (abs(targetSpeedRight) < MAX_PULSE_SPEED) {
        timerStateRight = TIMER_PULSE;
        Timer4.initialize(1800);
//        Timer4.initialize(tp4Ky);
      }
      MOTOR_PORT_RIGHT = MOTOR_BKWD;
    }
    else if (tickPeriodRight > targetBrakePeriodRight) { // Brake?
      MOTOR_PORT_RIGHT = MOTOR_BRAKE;
    }
    else { // coast
      MOTOR_PORT_RIGHT = MOTOR_COAST;
    }
  } // end BKWD
  else { // STOP
    MOTOR_PORT_RIGHT = MOTOR_BRAKE;
  }
} // end encoderIsrRight()



/******************* encoderIsrLeft() **************************/
void encoderIsrLeft() {
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  if (digitalRead(MOT_LEFT_ENCA) == digitalRead(MOT_LEFT_ENCB)) {
    tickPeriodLeft = (long) tickTimeLeft - (long) lastTickTime;
    tickDistanceLeft++;
  } 
  else {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickDistanceLeft--;
  }
  if (runState != STATE_RUNNING) {
    MOTOR_PORT_LEFT = MOTOR_COAST;
    return;
  } 
  if (mode == MODE_HOME) {
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
        Timer5.initialize(1800);
      }
      MOTOR_PORT_LEFT = MOTOR_FWD;
    }
    else if (tickPeriodLeft < targetBrakePeriodLeft) { // Brake?
      MOTOR_PORT_LEFT = MOTOR_BRAKE;
    } 
    else { // between target & brake.  Coast.
      MOTOR_PORT_LEFT = MOTOR_COAST;
    }
  } // end FWD
  else if (targetDirectionLeft == BKWD) {
    if ((tickPeriodLeft < targetTickPeriodLeft)  || (tickPeriodLeft >= 0)) { // Too slow?
      if (abs(targetSpeedLeft) < MAX_PULSE_SPEED) {
        timerStateLeft = TIMER_PULSE;
//        Timer5.initialize(tp4Ky);
        Timer5.initialize(1800);
      }
      MOTOR_PORT_LEFT = MOTOR_BKWD;
    }
    else if (tickPeriodLeft > targetBrakePeriodLeft) { // Brake?
      MOTOR_PORT_LEFT = MOTOR_BRAKE;
    }
    else { // coast
      MOTOR_PORT_LEFT = MOTOR_COAST;
    }
  } // end BKWD
  else { // STOP
    MOTOR_PORT_LEFT = MOTOR_BRAKE;
  }
} // end encoderIsrLeft()


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
    if (mode == MODE_HOME) {
      MOTOR_PORT_RIGHT = MOTOR_BRAKE;
      MOTOR_PORT_LEFT = MOTOR_BRAKE;
    }
    else {
      MOTOR_PORT_RIGHT = MOTOR_COAST; 
    }
    if (waitPeriodRight > 0) {
      Timer4.initialize(waitPeriodRight);
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
    MOTOR_PORT_LEFT = MOTOR_COAST;
    if (waitPeriodLeft > 0) {
      Timer5.initialize(waitPeriodLeft);
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
  if (mode == MODE_HOME) {
    return;
  }
  if (runState != STATE_RUNNING) {
    noInterrupts();
      MOTOR_PORT_RIGHT = MOTOR_COAST;
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
        Timer4.initialize(1800);
      }
      noInterrupts();
      MOTOR_PORT_RIGHT = MOTOR_FWD;
      interrupts();    
    }
    else if (targetDirectionRight == BKWD) {
      if (abs(targetSpeedRight) < MAX_PULSE_SPEED) {
        timerStateRight = TIMER_PULSE;
//        Timer4.initialize(tp4Ky);
        Timer4.initialize(1800);
      }
      noInterrupts();
      MOTOR_PORT_RIGHT = MOTOR_BKWD;
      interrupts();    
    }
    else { // STOP
      noInterrupts();
      MOTOR_PORT_RIGHT = MOTOR_BRAKE;
      interrupts();    
    }
  } 
}


/************************ checkMotorLeft *************************/
void checkMotorLeft() {
  if (mode == MODE_HOME) {
    return;
  }
  if (runState != STATE_RUNNING) {
    noInterrupts();
    MOTOR_PORT_LEFT = MOTOR_COAST;
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
        Timer5.initialize(1800);
      }
      noInterrupts();
      MOTOR_PORT_LEFT = MOTOR_FWD;
      interrupts();    
    }
    else if (targetDirectionLeft == BKWD) {
      if (abs(targetSpeedLeft) < MAX_PULSE_SPEED) {
        timerStateLeft = TIMER_PULSE;
//        Timer5.initialize(tp4Ky);
        Timer5.initialize(1800);
      }
      noInterrupts();
      MOTOR_PORT_LEFT = MOTOR_BKWD;
      interrupts();    
    }
    else { // STOP
      noInterrupts();
     MOTOR_PORT_LEFT = MOTOR_BRAKE;
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


