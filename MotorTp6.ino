//#define D_CONST 100  // The rate that interrupts occur at 100% duty cycle
//#define K_CONST 1    // Weight to give pulse width that deviates target fps
//#define H_POLL_P 50 // Half the poll period. e.g. 50 = 100 poll period
//
///**************************************************************************.
// *  motorInitTp6() 
// **************************************************************************/
//void motorInitTp6() {
//    // Initialize the pwm frequency
//  pwm_set_resolution(16);  
//  pwm_setup(MOT_RIGHT_PWMH, TP_PWM_FREQUENCY, 1);  // on clock A
//  pwm_setup(MOT_LEFT_PWMH, TP_PWM_FREQUENCY, 1);  // on clock A
//
//  // Set the pin modes
//  // Set the pin modes
//  pinMode(MOT_RIGHT_PWML, OUTPUT);
//  pinMode(MOT_RIGHT_DIR, OUTPUT);
//  pinMode(MOT_LEFT_PWML, OUTPUT);
//  pinMode(MOT_LEFT_DIR, OUTPUT);
//  
//  digitalWrite(MOT_RIGHT_PWML, LOW);
////  digitalWrite(MOT_RIGHT_PWMH, LOW);
//  digitalWrite(MOT_RIGHT_DIR, LOW);
////  digitalWrite(MOT_LEFT_PWML, LOW);
//  digitalWrite(MOT_LEFT_PWMH, LOW);
//  digitalWrite(MOT_LEFT_DIR, LOW);
//
//  setMotor(MOTOR_RIGHT, BRAKE, 0);
//  setMotor(MOTOR_LEFT, BRAKE, 0);
//
//  
//  targetMFpsRight = 0;
//  targetMFpsLeft = 0;
//  
//  attachInterrupt(MOT_RIGHT_ENCA, encoderIsr6Right, CHANGE);
//  attachInterrupt(MOT_LEFT_ENCA, encoderIsr6Left, CHANGE);
////  Timer0.attachInterrupt(pollIsr6);
////  Timer0.start(H_POLL_P * 2);
////  Timer1.attachInterrupt(timerIsrLeft);
//}
//
////void pollIsr6() {
////  int t = micros();
////  if ((stopTimeRight < t) && motorStateRight) {
////    g_APinDescription[MOT_RIGHT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWMH].ulPin; // LOW
////    g_APinDescription[MOT_RIGHT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWML].ulPin; // LOW
////    stopTimeRight = UNSIGNED_LONG_MAX;
////    motorStateRight = false;
////    unsigned int p = t - startTimeRight;
////    if (signPwRight > 0) pwPlusSumRight += p;
////    else pwMinusSumRight += p;
////  }
////  if ((stopTimeLeft < timeMicroseconds) && motorStateLeft) {
////    g_APinDescription[MOT_LEFT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWMH].ulPin; // LOW
////    g_APinDescription[MOT_LEFT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWML].ulPin; // LOW
////    stopTimeLeft = UNSIGNED_LONG_MAX;
////    motorStateLeft = false;
////  }
////}
////
//
///**************************************************************************.
// *
// * encoderIsr6Right()
// *
// *    Responds to interrupts from the encoder.
// *    Controls the motor directly from this isr.
// *
// ***************************************************************************/
//void encoderIsr6Right() {
//  boolean encAStat;
//  int pw = 0;
//  int signPw = 0;
//  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
//  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
//  if (encA == encAStat) {
//    interruptErrorsRight++;
//    return;  // Ignore if bogus interrupt!
//  }
//  encAStat = encA;
//  unsigned long lastTickTime = tickTimeRight;
//  tickTimeRight = micros();  
//    
//  if (encA == encB) {
//    tickPeriodRight = (long) tickTimeRight - (long) lastTickTime;
//    tickPositionRight++;
//  } 
//  else {
//    tickPeriodRight = (long) lastTickTime - (long) tickTimeRight;
//    tickPositionRight--;
//  }
//  wsMFpsRight = (ENC_FACTOR_M / tickPeriodRight); // speed in milli-fps
//  wsMFpsRightSum += wsMFpsRight; // TODO may not be needed
//  wsMFpsRightCount++;            // TODO may not be needed
//  
//  // Compute motor-on-time for log
//  if (motorStateRight) { 
//    if (signPwRight > 0) pwPlusSumRight += abs(tickPeriodRight);
//    else pwMinusSumRight += abs(tickPeriodRight);
//  }
//  
//  if (mode == MODE_PULSE_SEQUENCE) return;
//  if (mode == MODE_PWM_SPEED) return;
//  
//  // Set the motor direction and pulse width
//  if (targetMFpsRight > 0) pw = D_CONST;
//  else pw = -D_CONST;
//  
//  // compute K
//  rightK = (2000/abs(targetMFpsRight)) + 1;
//  if (rightK > 12) rightK = 12;
//  pw -= rightK * (wsMFpsRight - targetMFpsRight);
////pw = pw / 2;
//  signPw = pw;
//  if (pw >= 0) {
//    g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_DIR].ulPin;   // HIGH
//  }
//  else {
//    g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_DIR].ulPin;   // LOW
//    pw = -pw;
//  }
//  if (((tpState & TP_STATE_RUNNING) == 0) || (pw <= H_POLL_P)) {
//    g_APinDescription[MOT_RIGHT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWMH].ulPin; // LOW
//    g_APinDescription[MOT_RIGHT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWML].ulPin; // LOW
//    stopTimeLeft = UNSIGNED_LONG_MAX;
//    motorStateLeft = false;
//    stopTimeRight = UNSIGNED_LONG_MAX;
//  }
//  else {
//      g_APinDescription[MOT_RIGHT_PWMH].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_PWMH].ulPin; // HIGH
//      g_APinDescription[MOT_RIGHT_PWML].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_PWML].ulPin; // HIGH
//      motorStateRight = true;
//      startTimeRight = tickTimeRight;
//      stopTimeRight = tickTimeRight + pw - H_POLL_P;
//  }
//  
////addLog((long) (tickTimeRight),
////       (short) (tickPositionRight),
////       (short) (targetMFpsRight),
////       (short) (wsMFpsRight),
////       (short) (signPw), 
////       (short) (0),
////       (short) (0));
//} // encoderIsr6Right()
//
//
///**************************************************************************.
// *  encoderIsrLeft() 
// **************************************************************************/
//void encoderIsr6Left() {
//  boolean encAStat;
//  int pw = 0;
//  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
//  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
//  if (encA == encAStat) {
//    interruptErrorsLeft++;
//    return;  // Ignore if bogus interrupt!
//  }
//  encAStat = encA;
//  unsigned long lastTickTime = tickTimeLeft;
//  tickTimeLeft = micros();  
//    
//  if (encA != encB) {
//    tickPeriodLeft = (long) tickTimeLeft - (long) lastTickTime;
//    tickPositionLeft++;
//  } 
//  else {
//    tickPeriodLeft = (long) lastTickTime - (long) tickTimeLeft;
//    tickPositionLeft--;
//  }
//  wsMFpsLeft = (ENC_FACTOR_M / tickPeriodLeft); // speed in milli-fps
//  wsMFpsLeftSum += wsMFpsLeft; // TODO may not be needed
//  wsMFpsLeftCount++;            // TODO may not be needed
//  
//  if (mode == MODE_PULSE_SEQUENCE) return;
//  if (mode == MODE_PWM_SPEED) return;
//  
//  // Set the motor direction and pulse width
//  if (targetMFpsLeft > 0) pw = D_CONST;
//  else pw = -D_CONST;
//  
//  // compute K
//  leftK = (2000/abs(targetMFpsLeft)) + 1;
//  if (leftK > 12) leftK = 12;
//  pw -= leftK * (wsMFpsLeft - targetMFpsLeft);
////pw = pw / 2;
//  signPwRight = pw;
//  if (pw >= 0) {
//    g_APinDescription[MOT_LEFT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_DIR].ulPin;   // LOW
//  }
//  else {
//    g_APinDescription[MOT_LEFT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_DIR].ulPin;   // HIGH
//    pw = -pw;
//  }
//  if (((tpState & TP_STATE_RUNNING) == 0) || (pw <= H_POLL_P)) {
//    g_APinDescription[MOT_LEFT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWMH].ulPin; // LOW
//    g_APinDescription[MOT_LEFT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWML].ulPin; // LOW
//    motorStateLeft = false;
//  }
//  else {
//      g_APinDescription[MOT_LEFT_PWMH].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_PWMH].ulPin; // HIGH
//      g_APinDescription[MOT_LEFT_PWML].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_PWML].ulPin; // HIGH
//      motorStateLeft = true;
//      startTimeLeft = tickTimeLeft;
//      stopTimeLeft = tickTimeLeft + pw - H_POLL_P;
//  }
//} // end encoderIsr6Left();
//
///**************************************************************************.
// *  timerIsrXXXX() 
// **************************************************************************/
////void timerIsrRight() {
////  motorStateRight = false;
////  Timer0.stop();
////  g_APinDescription[MOT_RIGHT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWMH].ulPin; // LOW
////  g_APinDescription[MOT_RIGHT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWML].ulPin; // LOW
////  unsigned int t = micros() - tickTimeRight;
////  if (signPwRight > 0) pwPlusSumRight += t;
////  else pwMinusSumRight += t;
////}
////void timerIsrLeft() {
////  Timer1.stop();
////  g_APinDescription[MOT_LEFT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWMH].ulPin; // LOW 
////  g_APinDescription[MOT_LEFT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWML].ulPin; // LOW 
////}
//
//
//
//
//
//
///**************************************************************************.
// *
// * checkMotorXXX()
// *
// *    Starts the motor if idle
// *
// **************************************************************************/
//void checkMotor6Right() {
//  unsigned int t = micros();
//  unsigned int cTickPeriod = t - tickTimeRight;
//  if (cTickPeriod < 5000) return;
//  int cFpsRight = (ENC_FACTOR_M / cTickPeriod); // speed in milli-fps
//  if (abs((targetMFpsRight) / 2) > cFpsRight) {  // Check this!
//    if ((!isStateBit(TP_STATE_RUNNING)) || (targetMFpsRight == 0)) {
//      g_APinDescription[MOT_RIGHT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWMH].ulPin; // LOW
//      g_APinDescription[MOT_RIGHT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_PWML].ulPin; // LOW
//    }
//    else {
//      if (targetMFpsRight > 0) {
//        g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_DIR].ulPin;   // HIGH
//      }
//      else {
//        g_APinDescription[MOT_RIGHT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_RIGHT_DIR].ulPin;   // LOW    
//      }
//      g_APinDescription[MOT_RIGHT_PWMH].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_PWMH].ulPin; // HIGH
//      g_APinDescription[MOT_RIGHT_PWML].pPort -> PIO_SODR = g_APinDescription[MOT_RIGHT_PWML].ulPin; // HIGH
////      Timer0.start(1500);
//      startTimeRight = t;
////      stopTimeRight = t + 1500;
//      stopTimeRight = t + 800;
//      motorStateRight = true;
//    }
//  }
//}
///**************************************************************************.
// *  checkMotorLeft() 
// **************************************************************************/
//void checkMotor6Left() {
//  unsigned int t = micros();
//  int cTickPeriod = t - tickTimeLeft;
//  if (cTickPeriod < 5000) return;
//  int cFpsLeft = (ENC_FACTOR_M / cTickPeriod); // speed in milli-fps
//  if (abs((targetMFpsLeft) / 2) > cFpsLeft) {
//    if ((!isStateBit(TP_STATE_RUNNING)) || (targetMFpsLeft == 0)) {
//      g_APinDescription[MOT_LEFT_PWMH].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWMH].ulPin; // LOW
//      g_APinDescription[MOT_LEFT_PWML].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_PWML].ulPin; // LOW
//    }
//    else {
//      if (targetMFpsLeft > 0) {
//        g_APinDescription[MOT_LEFT_DIR].pPort -> PIO_CODR = g_APinDescription[MOT_LEFT_DIR].ulPin;   // LOW    
//      }
//      else {
//        g_APinDescription[MOT_LEFT_DIR].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_DIR].ulPin;   // HIGH
//      }
//      g_APinDescription[MOT_LEFT_PWMH].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_PWMH].ulPin; // HIGH
//      g_APinDescription[MOT_LEFT_PWML].pPort -> PIO_SODR = g_APinDescription[MOT_LEFT_PWML].ulPin; // HIGH
////      Timer1.start(1500);
//      startTimeLeft = t;
////      stopTimeLeft = t + 1500;
//      stopTimeLeft = t + 800;
//      motorStateLeft = true;
//    }
//  }
//}
