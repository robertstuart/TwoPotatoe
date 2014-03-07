/* ---------------------- TwoPotatoe ---------------------- */
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <stdlib.h>
#include "Common.h"
#include "TimerThree.h"
#include "TimerFour.h"
#include "TimerFive.h"

#define MOTOR_COAST B00000000
#define MOTOR_BRAKE B00000111
#define MOTOR_FWD   B00000110
#define MOTOR_BKWD  B00000101
#define MOTOR_PORT_RIGHT PORTA
#define MOTOR_PORT_LEFT PORTC



// set motor to test
const boolean TEST_RIGHT = true;
const boolean TEST_LEFT = true;

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_RIGHT_INA = 30;    // green from motor1 controller INA
#define MOT_RIGHT_INB 31    // blue from motor1 controller INB
#define MOT_RIGHT_ENCA 2   // yellow from right motor, encoder A, (interrupt0)
#define MOT_RIGHT_ENCB 42  // white from motor2, encoder B
#define MOT_LEFT_INA 40    // blue from motor 2 controller INA
#define MOT_LEFT_INB 41    // green from motor 2 controller ING
#define MOT_LEFT_ENCA 3   // yellow from left motor, encoder A, (interrupt1)
#define MOT_LEFT_ENCB 43  // white from motor1, encoder B 
#define MOT_RIGHT_PWM 4    // yellow from motor1 controller, pulse width signal
#define MOT_LEFT_PWM 5   // yellow from motor2 controller

#define BRAKE 3
#define COAST 2
#define FWD 1
#define STOP 0
#define BKWD -1
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2

#define WIXA_PIN 0 //Wixel RX.  DO NOT USE!
#define WIXB_PIN 1 //Wixel TX.  DO NOT USE!
#define LED_PIN 13 // LED connected to digital pin 13
#define GREEN_LED_PIN 50 // LED in switch
#define RED_LED_PIN 48 // LED in switch
#define PWR_PIN 49 // Mosfet power controller
#define AUDIO_PIN 52

//Encoder factor
//const float ENC_FACTOR= 750.0f;  // Change pulse width to fps speed, 1/50 gear
const float ENC_FACTOR = 1329.0f;  // Change pulse width to fps speed, 1/29 gear
const float FPS_TO_TPCS = 7.52f;   // Convert foot/sec to tics/centisecond
const float ENC_BRAKE_FACTOR = ENC_FACTOR * 0.95f;

// Max values for integers & longs
#define UNSIGNED_LONG_MAX 4294967295UL 
#define LONG_MAX  2147483647L
#define LONG_MIN -2147483648L
#define INT_MAX 32767

// Constants for IMU
#define GYRO_SENS 0.009375     // Multiplier to get degree. -0.075/8?
#define DRIFT_COUNT 100
#define ACCEL_SENS 0.24    // Multiplier to get degree
#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer
#define TICK_WEIGHT 0.98    // Weight for tick compared to accelerometer
#define TICKS_PER_DEGREE 30.3f
#define TICKS_PER_FOOT 1600
#define FPS_ANGLE_RATE (-TICKS_PER_FOOT / (TICKS_PER_DEGREE * 100.0f))
#define RADIANS_PER_DEGREE 0.0174
#define TICK_INTEGRATION_RATE .95
#define TA_FACTOR 5.8

// Battery definitions
const float BATT_ATOD_MULTIPLIER = 0.01387; // value to multiply atod output to get voltage
#define BATTERY_PIN A3              // Analog pin assignment
const int PRESSURE_PIN = A4;             // Pressure sensor

// Timer states
#define TIMER_PULSE 0   // Timer is in a pulse
#define TIMER_WAIT 1    // TImer is waiting after a pulse
#define TIMER_IDLE 2

#define MAX_PULSE_SPEED 0.8
//#define MAX_PULSE_WAIT 8000
//#define PULSE_LENGTH 1500

typedef struct {
  float t;
  float u;
  float v;
  float w;
  float x;
  float y;
  float z;
} valSet;

valSet tp4A = { 
                  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
                  0.0,    // u tick angle added in.  ~0-2.0. 
                  1.0,    // v rotation subtraction, 0-2.0?
                  0.2,    // w cos smoothing rate.  0-1.0
                  2.0,    // x CO speed error to angle factor
                  0.09,   // Y Target angle to WS
                  -3.15}; // z accelerometer offset
                  
valSet tp4B = { 
                  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
                  0.0,    // u tick angle added in.  ~0-2.0. 
                  1.0,    // v rotation subtraction, 0-2.0?
                  0.3,    // w cos smoothing rate.  0-1.0
                  3.0,    // x CO speed error to angle factor
                  0.15,   // Y Target angle to WS
                  -3.15}; // z accelerometer offset
                  
valSet tp4C = { 
                  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
                  0.0,    // u tick angle added in.  ~0-2.0. 
                  1.0,    // v rotation subtraction, 0-2.0?
                  0.3,    // w cos smoothing rate.  0-1.0
                  3.0,    // x CO speed error to angle factor
                  0.15,   // Y Target angle to WS
                  -3.15}; // z accelerometer offset
                  
valSet *currentValSet = &tp4A;
int valSetStat = VAL_SET_A;

  // general constants
//
//  // from A_TP4
//  float tp4Kv = 4.0;     // Speed error to angle
//  float tp4Kw = 0.18;    // Speed error to angle
//  float tp4Kx = 8000.0;     // max pulse wait
//  float tp4Ky = 1800.0;   // pulse lenght
////  float tp4Kx = 0.3;     // coSpeed integration
////  float tp4Ky = 0.87;   // base speed integration
//  float tp4Kz = 1800.0;   // home integrated error
//
//  // from A_HOME
//  float homeKv = 0.0f;
//  float homeKw = 0.0f;
//  float homeKx = 0.0f;
//  float homeKy = 0.0f;
//  float homeKz = 0.0f;
//  
  
int timerStateRight = TIMER_IDLE;
int timerStateLeft = TIMER_IDLE;

long debugCountA = 0;  //Incremented or set by various routines for debugging.
long debugCountB = 0;  //Incremented or set by various routines for debugging.
long debugCountC = 0;  //Incremented or set by various routines for debugging.
long debugCountD = 0;  //Incremented or set by various routines for debugging.

unsigned long t;

// Values from encoder interrupts
long resetTickPositionRight = 0L;
long resetTickPositionLeft = 0L;
unsigned long resetTimeRight;
unsigned long resetTimeLeft;

// Speed and position variables
long tickDistanceRight = 0L;
long tickDistanceLeft = 0L;
long tpPositionDiff = 0L;
long tickDistance;
float targetSpeedRight = 0.0;
float targetSpeedLeft = 0.0;
unsigned long tickTimeRight = 0UL;  // time for the last interrupt
unsigned long tickTimeLeft = 0UL;
long targetTickPeriodRight = 0L;
long targetBrakePeriodRight = 0L;
long targetTickPeriodLeft = 0L;
long targetBrakePeriodLeft = 0L;
long tickPeriodRight = 0L;     // last period. Minus for reverse.
long tickPeriodLeft = 0L;
float tickAngle = 0;
float fpsRight = 0.0f; // right feet per second
float fpsLeft = 0.0f;  // left feet per second
float tpcsRight = 0.0f; // right ticks per centi-second 
float tpcsLeft = 0.0f; // left ticks per centi-second 
float wheelSpeedFps = 0.0f;
float speedTpcs = 0.0f;

unsigned long waitPeriodRight = 0UL;  // Wait beyond beginning of pulse!!!
unsigned long waitPeriodLeft = 0UL;  // Wait beyond beginning of pulse!!!

int targetDirectionRight = FWD;
int targetDirectionLeft = FWD;

// debug flags and variables
boolean aDebug = false;
float aVal;
boolean bDebug = false;
float bVal;
boolean cDebug = false;
float cVal;
boolean dDebug = false;
float dVal;
boolean eDebug = false;
float eVal;
boolean fDebug = false;
float fVal;
boolean gDebug = false;
float gVal;
boolean hDebug = false;
float hVal;
boolean iDebug = false;
float iVal;
boolean jDebug = false;
float jVal;
boolean kDebug = false;
float kVal;

// Sequence variables
int sequenceCount = 0;
boolean sequenceIsRunning = false;
int runSequenceCount = 0;
boolean isSequence = false;
int seqDur = 0;
float seqWs = 0.0;
int seqPw = 0;
int pwArray[30];
float wsArray[30];
int durArray[30];


L3G gyro;
LSM303 compass;

// Public variables
unsigned int mode = MODE_TP4;
//boolean startup = true; // True intil TP reaches vertical for first time.
unsigned long loopTime = 11000L;     // Actual time set by Algorithm initXXX() routines.
unsigned long timeTrigger = 0L;
unsigned long oldTimeTrigger = 0L;
unsigned long timeMicroSeconds = 0L; // Set in main loop.  Used by several routines.
unsigned long aliveTrigger = 0L;
unsigned long lastHandshake = 0L;
boolean handshakeAlive = false;

int remotePwR = 0;
int remotePwL = 0;
float remoteTpR = 0.0f;
float remoteTpL = 0.0f;

float rawError = 0.0;
float pError = 0.0;
float rawIError = 0.0;
float iError = 0.0;
float dError = 0.0;
float pid = 0.0;

int tipCount = 0;
float ae = 0.0;

int runState = STATE_RESTING; // 
boolean sitting = true;  // sitting on the ground.
boolean upright = false;

long home = 0L;  // home tpPosition, zero indicates no home position.
unsigned int pressure = 0; // pressure sensor value
int pressureCount = 0;
unsigned int tpPw = 2000;
long tickRate; 
float integratedTickRate = 0.0f; 
int gravityCount = 0;

// Values computed before calling algorithm()
unsigned int actualLoopTime; // Time since the last
float loopSec;               // float value of above
float controllerX = 0.0; // +1.0 to -1.0 from controller
float controllerY = 0.0;  // Y value set by message from controller

int gyroXRaw;  // Vertical plane parallel to wheels
float gyroXRate;
float gyroXAngle = 0.0;
int gyroXDriftCount = 0;
long gyroXRawSum;
long gyroZRawSum;
int driftCount;
int driftX = 0;
int driftZ = 0;
float accelXAngle = 0.0;  // Vertical plane parallel to wheels
float gaXAngle = 0.0f;
float gatXAngle = 0.0f;

int gyroYRaw = 0;
float gyroYRate;
float gyroYAngle = 0.0f;
float accelYAngle = 0.0f;
float gaYAngle = 0.0f;

float gyroZRaw = 0.0f;
float gyroZRate = 0.0f;
float gyroZAngle = 0.0f;

long oldTickRate = 0;
float oldtpAngle1 = 0.0;
float tpAngle1Rate = 0.0;
boolean straightMode = false;

float coFallingAccel = 0.0f;;
float coFallingRate = 0.0f;
float coDistance = 0.0f;
float tpAngle2 = 0.0f;
float tpAngle2Rate = 0.0f;

float general = 42.42f;

float rotateTarget = 0.0;
boolean isRotating = false;

int pingHcTpCount = 0;
int pingPcTpCount = 0;
int pingTpCount = 0; // pings going out
unsigned long pingPcTpTime = 0L;
unsigned long pingHcTpTime = 0L;
unsigned int pingHcTpErrors = 0;
unsigned int pingPcTpErrors = 0;

int ackCount = 0;;
int missing =0;
long lastCount = 0;
unsigned long ackTrigger = 0L;
unsigned long thisCount;

unsigned int unknownPcSingleCmdErrors = 0;
unsigned int unknownPcParamCmdErrors = 0;
unsigned int unknownHcSingleCmdErrors = 0;
unsigned int unknownHcParamCmdErrors = 0;
unsigned int byteCountErrorsHc = 0;
unsigned int unknownPcSingleCmd = 0;
unsigned int unknownPcParamCmd = 0;

float batteryVoltage = 0.0f;
boolean hcAlive = false;
boolean pcAlive = false;
float gyroXAngleDelta = 0;

long oldTickDistance = 0;
int oldTickSpeed = 0;
float oldTickAngle = 0.0f;


/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {
  pinMode (LED_PIN,OUTPUT);  // Status LED
  pinMode (RED_LED_PIN,OUTPUT);  // Status LED
  pinMode (GREEN_LED_PIN,OUTPUT);  // Switch LED
  pinMode (PWR_PIN,OUTPUT);  // Power mosfet control
  pinMode (AUDIO_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  digitalWrite(BATTERY_PIN, LOW); // No pullup.  Necessary?

  Wire.begin();

  // TODO revisit these parameters
  compass.init(LSM303DLHC_DEVICE, 0);
  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth
//  gyro.writeReg(L3G_CTRL_REG2, 0x00); // 250 dps full scale
//  gyro.writeReg(L3G_CTRL_REG5, 0x10); // high-pass enable
//  gyro.writeReg(L3G_CTRL_REG2, 0x03); // high-pass frequency
//  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale

//  motorInit();
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(PWR_PIN, HIGH);
  
  //  Serial.begin(115200);
  Serial.begin(115200); // Wixel
  while (Serial.available()) {
    Serial.read();
  }
  Serial1.begin(57600); // Xbee
  while (Serial1.available()) {
    Serial1.read();
  }
  algorithmInit(MODE_TP4);
  timeTrigger = micros();
  tone(AUDIO_PIN, 880, 200);
} // end setup()



/*********************************************************
 *
 * loop()
 *
 *     Required by the Arduino system.  Called frequently, 
 *     hopefully at least every millisecond.
 *
 *********************************************************/
void loop() { //Main Loop
  readPCController();  // Read wixel connected to PC
  readHController();  // Read XBee on Arduino-based hand controller
  timeMicroSeconds = micros();
  
  // Start timed loop
  if(timeMicroSeconds > timeTrigger) {  // Loop executed every XX microseconds 
    actualLoopTime = timeMicroSeconds - oldTimeTrigger;
    loopSec = ((float) actualLoopTime)/1000000.0;
    timeTrigger += loopTime;
    oldTimeTrigger = timeMicroSeconds;
    checkAlive();
    getTpAngle();
    readSpeed();
    algorithm();  // Do the computations and run motors.
    monitor();  // Send out any values to controller
    tasks();
    checkDrift();
    checkState();
  } // end timed loop

  // The remaining calls are made every loop.
  if (     (mode == MODE_TP4) 
    || (mode == MODE_TP_SPEED ) 
    || (mode == MODE_TP3 ) 
    || (mode == MODE_TP_SEQUENCE)) {
    checkMotorRight();
    checkMotorLeft();
  } 
} // End loop().  



/*********************************************************
 *
 * getTpAngle()
 *
 *     Computes the angle from the accelerometer and the gyro.
 *     Sets: gyroRate
 *           tpAngle
 *
 *********************************************************/
float sumGyroRate;
float getTpAngle() {
  gyro.read();   
  compass.readAcc();  
  
  // Compute angle around the x axis
  gyroXRaw = gyro.g.x;  // 
  gyroXRate = (gyroXRaw - driftX) * GYRO_SENS;  // Rate in degreesChange/sec
  gyroXAngleDelta = (gyroXRate * actualLoopTime)/1000000; // degrees changed during period
  gyroXAngle = gyroXAngle + gyroXAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroXWeightedAngle = gyroXAngleDelta + gaXAngle;  // used in weighting final angle
  accelXAngle = ((atan2(-compass.a.y, compass.a.z))*-RAD_TO_DEG) + (*currentValSet).z;  // angle from accelerometer
  constrain(accelXAngle, -90.0f, 90.0f);
  gaXAngle = (gyroXWeightedAngle * GYRO_WEIGHT) + (accelXAngle * (1 - GYRO_WEIGHT)); // Weigh factors  
  
  // Add in tick data to improve angle measurement
  long tickDistance = tickDistanceRight + tickDistanceLeft;
  int tickSpeed = (tickDistance - oldTickDistance);
  tickAngle = (oldTickSpeed - tickSpeed) + ((*currentValSet).t * oldTickAngle);
  oldTickDistance = tickDistance;
  oldTickSpeed = tickSpeed;
  oldTickAngle = tickAngle;
  gatXAngle = (tickAngle * (*currentValSet).u) + gaXAngle;
  
  // compute the Y plane to check for falling sideways
  gyroYRaw = -gyro.g.y;
  gyroYRate = gyroYRaw * GYRO_SENS;
  float gyroYAngleDelta = (gyroYRate * actualLoopTime)/1000000;
  gyroYAngle = gyroYAngle + gyroYAngleDelta; // not used
  float gyroYWeightedAngle = gyroYAngleDelta + gaYAngle;
  accelYAngle = atan2(compass.a.x, compass.a.z) * RAD_TO_DEG;
  gaYAngle = (gyroYWeightedAngle * GYRO_WEIGHT) + (accelYAngle * (1 - GYRO_WEIGHT));
  
  // compute Z plane to measure turns
  gyroZRaw = -gyro.g.z;
  gyroZRate = (gyroZRaw - driftZ) * GYRO_SENS;
  float gyroZAngleDelta = (gyroZRate * actualLoopTime)/1000000;
  gyroZAngle = gyroZAngle + gyroZAngleDelta; 
 
}




/*********************************************************
 *
 * getState()
 *
 *     Move the state between STATE_READY and STATE_RUN
 *     depending on the state of "upright" and "sitting".
 *     Always put in running mode if in a test state.
 *
 *********************************************************/
void checkState() {
  // Check if in one of the "run" modes.
  if ((mode == MODE_TP4) || (mode == MODE_HOME)) {
    if ((runState == STATE_READY) && upright && sitting) {
        runState = STATE_RUNNING;
        rawIError = 0;
        algorithmStart();
    }
    else if (runState == STATE_RUNNING) {
      if (!upright || !sitting) {
        runState = STATE_READY;
      }
    }
  }
  else { // Otherwise, in a "test" mode so we don't look at upright & sitting
    runState = STATE_RUNNING;
  }
}



/*********************************************************
 *
 * algorithm()
 *
 *     Call the correct processing routine for the mode we are in.
 *
 *********************************************************/
void algorithm() {
  switch (mode) {
  case MODE_MOTOR_PW:
//      setPwmSpeed(MOTOR_RIGHT, remotePwR);
//      setPwmSpeed(MOTOR_LEFT, remotePwL);
      aVal = fpsRight;
      bVal = fpsLeft;
      cVal = tickDistanceRight;
      dVal = tickDistanceLeft;
    break;
  case MODE_TP4:
    aTp4(); 
    break;
  case MODE_HOME:
    aHome();
    break;
  case MODE_TP_SEQUENCE:
    runTpSequence();
    break;
  case MODE_TP_SPEED:
      setTargetSpeedRight(remoteTpR);
      setTargetSpeedLeft(remoteTpL);
      aVal = fpsRight;
      bVal = fpsLeft;
      cVal = (float) MOTOR_PORT_RIGHT;
      dVal = tickDistanceLeft;
      eVal = tickPeriodRight;

    break;
  } // end switch  
} // end algorithm()


/*********************************************************
 *
 * checkDrift()
 *
 *     read gyro for DRIFT_COUNT centiseconds to cancel out drift;
 *
 *********************************************************/
void checkDrift() {
  if (driftCount > 0) {
    gyroXRawSum += gyroXRaw;
    gyroZRawSum += gyroZRaw;
    driftCount--;
    if (driftCount == 0) {
      driftX = gyroXRawSum / DRIFT_COUNT;
      debugFloat("X Drift/sec: ", driftX * GYRO_SENS);
      driftZ = gyroZRawSum / DRIFT_COUNT;
      debugFloat("Z Drift/sec: ", driftZ * GYRO_SENS);
    }  
  }
}



/*********************************************************
 *
 * checkAlive()
 *
 *     Check to see if the connection is alive.   *
 *********************************************************/
void checkAlive() {
  hcAlive = (pingHcTpTime + 500000L) > timeMicroSeconds;
  pcAlive = (pingPcTpTime + 500000L) > timeMicroSeconds;
  if (!hcAlive && !pcAlive) {
    controllerX = 0.0;
    controllerY = 0.0;
  }
}



/*********************************************************
 *
 * runPwmSequence()
 *
 *     Run the sequence already set by the LOQD command.
 *
 *********************************************************/
void initPwmSequence() {
    sequenceIsRunning = true;
    runSequenceCount = 0;
    seqDur = durArray[0];
    seqPw = pwArray[0];
}
void runPwmSequence() {
  if (!sequenceIsRunning) {
    return;
  }
  if (--seqDur < 0) { // move on to next pair?
    if (++runSequenceCount >= sequenceCount) { // Last one?
//      setPwmSpeed(MOTOR_RIGHT, 0);
//      setPwmSpeed(MOTOR_LEFT, 0);
      sequenceIsRunning = false;
      cmdSinglePC(CMD_SEQUENCE_END, false);
      return;
    }
    seqDur = durArray[runSequenceCount];
    seqPw = pwArray[runSequenceCount];
  }
  if (TEST_RIGHT) {
//    setPwmSpeed(MOTOR_RIGHT, seqPw);
    bVal = fpsRight;
  }
  if (TEST_LEFT) {
//    setPwmSpeed(MOTOR_LEFT, seqPw);
    bVal = fpsLeft;
  }
  if (TEST_RIGHT && TEST_LEFT) {
    bVal = wheelSpeedFps;
  }
  cVal = seqPw;
}



/*********************************************************
 *
 * runTpSequence()
 *
 *     Run the WS (Wheel Speed) sequence already set by the LOAD command.
 *
 *********************************************************/
void initTpSequence() {
    sequenceIsRunning = true;
    runSequenceCount = 0;
    seqDur = durArray[0];
    seqWs = wsArray[0];
    gyroXAngle = 0.0f;
    tickDistanceRight = tickDistanceLeft = 0L;
}
void runTpSequence() {
  if (!sequenceIsRunning) {
    seqWs = 0.0;
  }
  else if (--seqDur < 0) { // move on to next pair?
    if (++runSequenceCount >= sequenceCount) { // Last one?
      seqWs = 0.0;
      sequenceIsRunning = false;
      cmdSinglePC(CMD_SEQUENCE_END, false);
    } 
    else { // Not done.  Load next in sequence
      seqDur = durArray[runSequenceCount];
      seqWs = wsArray[runSequenceCount];
      debugFloat("seqWs: ", seqWs);
      debugInt("seqDur: ", seqDur);
    }
  } 
  
  setTargetSpeedRight(seqWs);    
  setTargetSpeedLeft(seqWs);

  aVal = timeMicroSeconds/1000;
  bVal = seqWs;
  cVal = wheelSpeedFps;
  dVal = gyroXAngle;
  eVal = accelXAngle;
  fVal = gaXAngle;
  gVal = tickDistanceRight + tickDistanceLeft;
}




/*********************************************************
 *
 * algorithmInit()
 *
 *     Called once when the mode is established via the 
 *     MODE command.
 *
 *********************************************************/
void algorithmInit(int x) {
  debugOff();  // Turn off all debugging flags.
  mode = x;
  runState = STATE_RESTING;
  debugInt("Starting mode: ", mode); // Seems to need this.
  driftCount = DRIFT_COUNT;
  gyroXRawSum = 0L;
  gyroZRawSum = 0L;
  
  switch (mode) {
  case MODE_MOTOR_PW:
    motorInitPwm();
    setPwmSpeed(MOTOR_RIGHT, 0);
    setPwmSpeed(MOTOR_LEFT, 0);
    break;
  case MODE_DRIVE:
    motorInitTp();
    break;
  case MODE_TP_SEQUENCE:
    motorInitTp();
    break;
  case MODE_TP4:
    aTp4Init();  
    break;
  case MODE_HOME:
    aHomeInit();
    break;
  case MODE_TP_SPEED:
    motorInitTp();
    break;
  } // end switch  
}  // end algorithmInit()




/*********************************************************
 *
 * algorithmStart()
 *
 *     Called when tp enters run state because run button
 *     is pressed or tp is vertical and resting on ground.
 *
 *********************************************************/
void algorithmStart() {
    
  gyroXAngle = gaXAngle;
  tickAngle = gaXAngle;
  tpAngle2 = gaXAngle;
  coDistance = 0.0f;
  tickDistance = -(tpAngle2 * TICKS_PER_DEGREE);
  tickDistanceRight = tickDistance/2;
  tickDistanceLeft = tickDistance/2;
  coFallingAccel = TA_FACTOR * sin(DEG_TO_RAD * tpAngle2);  
  coFallingRate = 0.0f;
  coDistance = 0.0f;
  tpAngle2Rate = 0.0f;
  tipCount = 0;
  cmdIntPC(CMD_RUN_STATE_STAT, runState);
  
  switch (mode) {
  case MODE_MOTOR_PW:
    break;
  case MODE_DRIVE:
    break;
  case MODE_TP4:
    aTp4Start();
    break;
  case MODE_HOME:
    aHomeStart();
    break;
  case MODE_TP_SPEED:
//    resetTp();
    break;
  case MODE_TP_SEQUENCE:
//    resetTp();
    break;
  } // end switch  
}


