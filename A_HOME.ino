/************************************************************************
 * ----- HOME -----
 * This routine hold in the "home" position when in the TP4 mode.
 * 
 * 
 ************************************************************************/
#define PID1_LOOP_TIME 10000L;        // Microsecons/loop

void aHomeInit() {
}

void aHomeStart() {
  home = tickDistanceRight + tickDistanceLeft;  
}

void aHome() {
  int homeMotorDirection = MOTOR_FWD;
  
  long homePosition = tickDistanceRight + tickDistanceLeft;
//  float homeTargetAngle = (home - homePosition) * homeKv;
//  float homeAngleError = gaXAngle - homeTargetAngle;
//  float homePulseWidth = homeAngleError * homeKw;
//  if (homePulseWidth < 0.0f) {
//    homeMotorDirection = MOTOR_BKWD;
//  }
  
  timerStateRight = TIMER_PULSE;
  waitPeriodRight = 0;
  if (runState == STATE_RUNNING) {
    MOTOR_PORT_RIGHT = homeMotorDirection;
    MOTOR_PORT_LEFT = homeMotorDirection; 
  }
//  Timer4.initialize(abs(homePulseWidth));
  
//  aVal = tickDistance;
//  bVal = gaXAngle;
//  cVal = homeTargetAngle;
//  dVal = homeAngleError;
//  eVal = homePulseWidth;

}

