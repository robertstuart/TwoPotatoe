#define NZ_PERIOD 6645 // 0.2fps, 1329 * 5
#define DAMP_THRESHOLD_A 999 // Accelerate: point to stop passive braking and start full reverse
#define DAMP_THRESHOLD_B 999 // Brake: point to stop passive braking and start full reverse
#define DAMP_THRESHOLD_NZ 999 // Brake: point to stop passive braking and start full reverse
#define CE_THRESHOLD 999 // Center of oscillation error
#define BRAKE_THRESHOLD 500 // 0.5 fps
#define ZTICK_DISTANCE_THRESHOLD 4
#define NZ_THRESHOLD 500  // 0.5 fps for near-zero speed threshold

////////////////////////////////////////////////////////
//debugValA = tickDistanceError;                      //
//debugValB = tickDistanceError;                      //
//debugValC = tickDistanceError;                      //
//debugValD = tickDistanceError;                      //
//debugValE = tickDistanceError;                      //
////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//tp7Log(dampValue, mFpsRight, tickDistanceError, action);     //
/////////////////////////////////////////////////////////////////

void motorInitTp7() {
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

  attachInterrupt(MOT_RIGHT_ENCA, isr7Right, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, isr7Left, CHANGE);
}

/**************************************************************************.
 *
 * isr7Right()
 *
 *    Responds to interrupts from the encoder.
 *    Controls the motor directly from this isr.
 *
 **************************************************************************/
void isr7Right() {
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
  int mWsFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
  
  // Compute where the tick distance should be at this point.
  unsigned long targetTickDistanceR =  baseTargetTickDistanceRight + ((tickTimeRight - baseTickTimeRight) / cosTickPeriodRight);
  
  // Get the error.
  long tickDistanceError = targetTickDistanceR - tickDistanceRight;
  if (tickDistanceError == 0) tickDistanceError = 1; // divide by zero?

  int dampValue = (mWsFpsRight - mLpfCos) / tickDistanceError;
  
  int action = motorControl(dampValue, mWsFpsRight, tickDistanceError);
  setMotor(MOTOR_RIGHT, action, 255);
}


/**************************************************************************.
 *
 * isr7Left()
 *
 *    Responds to interrupts from the encoder.
 *    Controls the motor directly from this isr.
 *
 **************************************************************************/
void isr7Left() {
 unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();  
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
    
  if (encA != encB) {
    tickPeriodLeft = (long) tickTimeLeft - (long) lastTickTime;
    tickDistanceLeft++;
  } 
  else {
    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
    tickDistanceLeft--;
  }
  if ((tpState & TP_STATE_RUNNING) == 0) {
    setMotor(MOTOR_LEFT, BRAKE, 0);
    return;
  }
  int mWsFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
  
  // Compute where the tick distance should be at this point.
  unsigned long targetTickDistanceL =  baseTargetTickDistanceLeft + ((tickTimeLeft - baseTickTimeLeft) / cosTickPeriodLeft);
  
  // Get the error.
  long tickDistanceError = targetTickDistanceL - tickDistanceLeft;
  if (tickDistanceError == 0) tickDistanceError = 1; // divide by zero?

  int dampValue = (mWsFpsLeft - mLpfCos) / tickDistanceError;
  
  int action = motorControl(dampValue, mWsFpsLeft, tickDistanceError);
  setMotor(MOTOR_LEFT, action, 255);
} // end isr7Left()


/**************************************************************************.
 * motorControl() Code explained in -----
 **************************************************************************/
int motorControl(int dampValue, int mWsFps, long tickDistanceError) {
  int cosError = abs(mLpfCos - mWsFps);
  int action = BRAKE;
  
  if (mWsFps > NZ_THRESHOLD) { 
    if (tickDistanceError > 0) {
      if (dampValue > DAMP_THRESHOLD_A) action = BKWD;
      else action = nz(tickDistanceError, cosError);
    } else {
      if (dampValue > DAMP_THRESHOLD_B) action = nz(tickDistanceError, cosError);
      else action = BKWD;
    }
  } else if (mWsFps < -NZ_THRESHOLD) {
    if (tickDistanceError > 0) {
      if (dampValue > DAMP_THRESHOLD_B) action = nz(tickDistanceError, cosError);
      else action = FWD;
    } else {
      if (dampValue > DAMP_THRESHOLD_A) action = FWD;
      else action = nz(tickDistanceError, cosError);
    }
  } else {
    if (tickDistanceError > 0) {
      if (dampValue > DAMP_THRESHOLD_NZ) action BKWD;
      else action = FWD;
    } else {
      if (dampValue > DAMP_THRESHOLD_NZ) action = FWD;
      else action = BKWD;
    }
    // Pulse width
    
  }  
  return action;
} // end motorControl()

int nz(int tde, int ce) {
//  if (abs(tde) > TICK_DISTANCE_THRESHOLD) return BRAKE;
//  if (ce > CE_THRESHOLD) return BRAKE;
//  return COAST;
return BRAKE; // make into noop initially
}
int nz(int tde, int ce, int action) {
  if (abs(tde) > ZTICK_DISTANCE_THRESHOLD) return action;
  if (ce > CE_THRESHOLD) return action;
  return BRAKE;
}

/**************************************************************************.
 * checkTp7MotorXXXX()
 **************************************************************************/
void checkTp7MotorRight() {
  if (timeMicroseconds > (tickTimeRight + 200000UL)) {
    long tickDistanceError = baseTargetTickDistanceRight - tickDistanceRight;
    if (tickDistanceError > 0L)   setMotor(MOTOR_RIGHT, FWD, 255);
    else   setMotor(MOTOR_RIGHT, BKWD, 255);
  }
}
void checkTp7MotorLeft() {
  if (timeMicroseconds > (tickTimeLeft + 200000UL)) {
    long tickDistanceError = baseTargetTickDistanceLeft - tickDistanceLeft;
    if (tickDistanceError > 0L)   setMotor(MOTOR_LEFT, FWD, 255);
    else   setMotor(MOTOR_LEFT, BKWD, 255);
  }
}



/**************************************************************************.
 * motorTp6XXXX()
 **************************************************************************/
void motorTp6Right(int tickError) {
//  unsigned long tEst = micros() - tickTimeRight;
//  long mWs = ENC_FACTOR_M /tickPeriodRight;
//  
//  // Use current time to compute speed if there is a long wait for an interrupt
//  if ((t - tickTimeRight) > abs(tickPeriodRight) {
//    if (mWs > 0) mWs = ENC_FACTOR_M /tickPeriodRight;
//    else mWs = -(ENC_FACTOR_M /tickPeriodRight);
//  }
//  
//  int dampValue = (mWs - mLpfCos) / tickError;
//  
//  int accelValue = tickError +  x * (mWs - mLpfCos);
//  
}
void motorTp6Left(int tickError) {
  
}

