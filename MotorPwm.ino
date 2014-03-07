//  This motor routine simply sets the pw of the motor.
//  The low values translate as follows:
//     +2 FWD with zero pw value
//     +1 brake high
//      0 undefined
//     -1 brake low
//     -2 BKWD with zero pw value
//  All other values are in the same direction so that
//  a pw of 255 requres and input value of 257


long threshRight = 0;
long threshLeft = 0;
/*********************************************************
 *
 * motorInit()
 *
 *    Set motor to initial state when the arduino
 *    starts for the first time.
 *
 *********************************************************/
void motorInit() {
  // set the encoder pins that are connected to an interrupt
  pinMode(MOT_RIGHT_ENCA, INPUT);  // interrupt
  pinMode(MOT_LEFT_ENCA, INPUT);  // interrupt

  // Set the remaining encoder pins  
  pinMode(MOT_RIGHT_ENCB, INPUT);
  pinMode(MOT_LEFT_ENCB, INPUT);

  // Set the INA and INB control pins
  pinMode(MOT_RIGHT_INA, OUTPUT);
  pinMode(MOT_RIGHT_INB, OUTPUT);
  pinMode(MOT_LEFT_INA, OUTPUT);
  pinMode(MOT_LEFT_INB, OUTPUT);

  // Set the Pulse width modulation pins
  pinMode(MOT_RIGHT_PWM, OUTPUT);
  pinMode(MOT_LEFT_PWM, OUTPUT);

  // Set the motors to coast
  digitalWrite(MOT_RIGHT_INA, LOW);
  digitalWrite(MOT_RIGHT_INB, LOW); 
  digitalWrite(MOT_RIGHT_PWM, LOW); 
  digitalWrite(MOT_LEFT_INA, LOW);
  digitalWrite(MOT_LEFT_INB, LOW); 
  digitalWrite(MOT_LEFT_PWM, LOW); 
}



/*********************************************************
 *
 * motorInitPwm()
 *
 *     Set the motor to run in pulsewidth mode.
 *
 *********************************************************/
void motorInitPwm() {
  attachInterrupt(0, interruptPwmRight, CHANGE);
  attachInterrupt(1, interruptPwmLeft, CHANGE);
}


/*********************************************************
 *
 * interruptPwmXXX()
 *
 *     Called when encoder interrupt occurs in pwm mode.
 *     Sets the encoder period.
 *
 *********************************************************/
void interruptPwmRight() {
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
}

void interruptPwmLeft() {
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
}





/*********************************************************
 *
 * setPwmSpeed(int motor, int pw)
 *
 *     Sets the motor to a give pulsewidth value.
 *
 *********************************************************/
void setPwmSpeed(int motor, int pw) {
  int pwm;
  int ina;
  int inb;

  pw = constrain(pw, -257, +257); // constrain
  if (motor == MOTOR_RIGHT) { // Right motor
    pwm = MOT_RIGHT_PWM;
    ina = MOT_RIGHT_INA; 
    inb = MOT_RIGHT_INB;
  } 
  else { // left motor
    pwm = MOT_LEFT_PWM;
    ina = MOT_LEFT_INA;
    inb = MOT_LEFT_INB;
  }

  // turn off motors if in halt state
  if(runState != STATE_RUNNING) {
    digitalWrite(ina, LOW); // Brake
    digitalWrite(inb, LOW);
    analogWrite(pwm, 0);
    return;
  } 

  if (pw == 1) {
    digitalWrite(ina, HIGH);  // hard brake. both directions
    digitalWrite(inb, HIGH);
    digitalWrite(pwm, HIGH);
  } 
  else if ((pw == -1) || (pw == 0)) {
    digitalWrite(ina, LOW);  // Coast, both directions
    digitalWrite(inb, LOW);
    digitalWrite(pwm, LOW);
  }
  else {
    if (pw > 0) { 
      digitalWrite(ina, LOW); // foreward
      digitalWrite(inb, HIGH);
      pw -= 2;
    } 
    else {
      digitalWrite(ina, HIGH);  // backward
      digitalWrite(inb, LOW);
      pw += 2;
    }
    analogWrite(pwm, abs(pw)); // duty cycle 0-255 (2-257)
  } 
}



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
  tpcsRight = FPS_TO_TPCS * fpsRight;
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
  tpcsLeft = FPS_TO_TPCS * fpsLeft;
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
  speedTpcs = (tpcsRight + tpcsLeft)/2.0f;
}

