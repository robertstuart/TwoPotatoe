
void motorInitTp7() {
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


