/* ---------------------- TwoPotatoe ---------------------- */
//#include <Wire.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//#include <L3G.h>
//#include <LSM303.h>
#include <stdlib.h>
#include "Common.h"
//#include "TimerThree.h"
//#include "TimerFour.h"
//#include "TimerFive.h"
#include <DueTimer.h>


// set motor to test
const boolean TEST_RIGHT = true;
const boolean TEST_LEFT = true;

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_RIGHT_INA = 40;    // green from motor1 controller INA
#define MOT_RIGHT_INB 41    // blue from motor1 controller INB
#define MOT_RIGHT_ENCA 42   // yellow from right motor, encoder A, (interrupt0)
#define MOT_RIGHT_ENCB 43  // white from motor2, encoder B
#define MOT_RIGHT_PWM 2    // yellow from motor1 controller, pulse width signal
#define MOT_LEFT_INA 46    // blue from motor 2 controller INA
#define MOT_LEFT_INB 47    // green from motor 2 controller ING
#define MOT_LEFT_ENCA 48   // yellow from left motor, encoder A, (interrupt1)
#define MOT_LEFT_ENCB 49  // white from motor1, encoder B 
#define MOT_LEFT_PWM 3   // yellow from motor2 controller

#define BRAKE 2
#define FWD 1
#define COAST 0
#define BKWD -1
#define STOP -2  // for target direction
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2

#define LED_PIN 13 // LED connected to digital pin 13
#define BLUE_LED_PIN 13 // LED in switch, same as status above
#define YELLOW_LED_PIN 12 // LED in switch
#define RED_LED_PIN 11 // LED in switch
#define RIGHT_HL_PIN 10
#define LEFT_HL_PIN 9
#define REAR_BL_PIN 8

#define YE_SW_PIN 24
#define PWR_PIN 25 // Mosfet power controller
#define SPEAKER_PIN 26

#define BATTERY_PIN A1              // Analog pin assignment
#define PRESSURE_PIN A0             // Pressure sensor

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
//#define INVEN_GYRO_SENS 0.009375     // Multiplier to get degree. -0.075/8?
#define DRIFT_COUNT 100
#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer
#define TICK_WEIGHT 0.98    // Weight for tick compared to accelerometer
#define TICKS_PER_DEGREE 30.3f
#define TICKS_PER_FOOT 1600
#define FPS_ANGLE_RATE (-TICKS_PER_FOOT / (TICKS_PER_DEGREE * 100.0f))
#define RADIANS_PER_DEGREE 0.0174
#define TICK_INTEGRATION_RATE .95
#define TA_FACTOR 5.8

// Battery definitions
const float BATT_ATOD_MULTIPLIER = 0.01525; // value to multiply atod output to get voltage

// Timer states
#define TIMER_PULSE 0   // Timer is in a pulse
#define TIMER_WAIT 1    // TImer is waiting after a pulse
#define TIMER_IDLE 2

#define MAX_PULSE_SPEED 0.8

#define END_MARKER 42
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
                  -2.55}; // z accelerometer offset
                  
valSet tp4B = { 
                  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
                  0.0,    // u tick angle added in.  ~0-2.0. 
                  1.0,    // v rotation subtraction, 0-2.0?
                  0.3,    // w cos smoothing rate.  0-1.0
                  3.0,    // x CO speed error to angle factor
                  0.15,   // Y Target angle to WS
                  -2.55}; // z accelerometer offset
                  
valSet tp4C = { 
                  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
                  0.0,    // u tick angle added in.  ~0-2.0. 
                  1.0,    // v rotation subtraction, 0-2.0?
                  0.3,    // w cos smoothing rate.  0-1.0
                  3.0,    // x CO speed error to angle factor
                  0.15,   // Y Target angle to WS
                  -2.55}; // z accelerometer offset
                  
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
boolean debugFlagArray[10] = {false,false,false,false,false,false,false,false,false,false};
long streamValueArray[10] = {0,0,0,0,0,0,0,0,0,0};

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
int wsDurArray[30];


//L3G gyro;
//LSM303 compass;
MPU6050 accelgyro;

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

boolean mtRun = false;
int unknownCmdErrors = 0;

int lamp;
boolean isStreaming = false;
boolean isPcConnected = false;
boolean isHcConnected = false;
int monitorCount = 0;

/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {
  
  //  Serial.begin(115200);
  Serial.begin(57600); // XBee
  
  pinMode(LED_PIN,OUTPUT);  // Status LED, also blue LED
  pinMode(YELLOW_LED_PIN,OUTPUT);
  pinMode(RED_LED_PIN,OUTPUT);
  pinMode(RIGHT_HL_PIN, OUTPUT);
  pinMode(LEFT_HL_PIN, OUTPUT);
//  pinMode(REAR_BL_PIN, OUTPUT);
  pinMode(PWR_PIN,OUTPUT);  // Power mosfet control
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(YE_SW_PIN, INPUT_PULLUP);
  
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(RIGHT_HL_PIN, LOW);
  digitalWrite(LEFT_HL_PIN, LOW);
  analogWrite(REAR_BL_PIN, 0);
  digitalWrite(SPEAKER_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  Wire.begin();

  delay(100);
  digitalWrite(PWR_PIN, HIGH);
  algorithmInit(MODE_TP4);
  timeTrigger = micros();
  beep(0,0);
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
  if (mtRun) {
    a_mt();
    return;
  }
  readXBee();  // Read XBee on Arduino-based hand controller
  timeMicroSeconds = micros();
  
  // Start timed loop
  if(timeMicroSeconds > timeTrigger) {  // Loop executed every XX microseconds 
    actualLoopTime = timeMicroSeconds - oldTimeTrigger;
    loopSec = ((float) actualLoopTime)/1000000.0;
    timeTrigger += loopTime;
    oldTimeTrigger = timeMicroSeconds;
    
    readSpeed();
    algorithm();  // Do the computations and run motors.
    if (isStreaming || (monitorCount++ > 5)) {
      monitorCount = 0;
      monitor();  // Send out values & query
    }
    tasks();
//    checkDrift();
//    checkState();
  } // end timed loop

  // The remaining calls are made every loop.
//  if (     (mode == MODE_TP4) 
//    || (mode == MODE_TP_SPEED ) 
//    || (mode == MODE_TP_SEQUENCE)) {
//    checkMotorRight();
//    checkMotorLeft();
//  } 
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
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

//  gyro.read();   
//  compass.readAcc();  
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Compute angle around the x axis
//  gyroXRaw = gyro.g.x;  // 
  gyroXRaw = gx;  // 
  gyroXRate = gyroXRaw * GYRO_SENS;  // Rate in degreesChange/sec
  gyroXAngleDelta = (gyroXRate * actualLoopTime)/1000000; // degrees changed during period
  gyroXAngle = gyroXAngle + gyroXAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroXWeightedAngle = gyroXAngleDelta + gaXAngle;  // used in weighting final angle
//  accelXAngle = ((atan2(-compass.a.y, compass.a.z))*-RAD_TO_DEG) + (*currentValSet).z;  // angle from accelerometer
  accelXAngle = ((atan2(-ay, az))*-RAD_TO_DEG) + (*currentValSet).z;  // angle from accelerometer
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
//  gyroYRaw = -gyro.g.y;
  gyroYRaw = gy;
  gyroYRate = gyroYRaw * GYRO_SENS;
  float gyroYAngleDelta = (gyroYRate * actualLoopTime)/1000000;
  gyroYAngle = gyroYAngle + gyroYAngleDelta; // not used
  float gyroYWeightedAngle = gyroYAngleDelta + gaYAngle;
//  accelYAngle = atan2(compass.a.x, compass.a.z) * RAD_TO_DEG;
  accelYAngle = atan2(ax, az) * RAD_TO_DEG;
  gaYAngle = (gyroYWeightedAngle * GYRO_WEIGHT) + (accelYAngle * (1 - GYRO_WEIGHT));
  
  // compute Z plane to measure turns
//  gyroZRaw = -gyro.g.z;
  gyroZRaw = -gz;
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
  if (mode == MODE_TP4) {
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
      streamValueArray[0] = fpsRight;
      streamValueArray[1] = fpsLeft;
      streamValueArray[2] = tickDistanceRight;
      streamValueArray[3] = tickDistanceLeft;
    break;
  case MODE_TP4:
    aTp4(); 
    break;
  case MODE_IMU:
    aImu();
    break;
  case MODE_TP_SEQUENCE:
    runTpSequence();
    break;
  case MODE_TP_SPEED:
      setTargetSpeedRight(remoteTpR);
      setTargetSpeedLeft(remoteTpL);
      streamValueArray[0] = fpsRight;
      streamValueArray[1] = fpsLeft;
      streamValueArray[2] = 0;
      streamValueArray[3] = tickDistanceLeft;
      streamValueArray[4] = tickPeriodRight;

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
//      debugFloat("X Drift/sec: ", driftX * GYRO_SENS);
      driftZ = gyroZRawSum / DRIFT_COUNT;
//      debugFloat("Z Drift/sec: ", driftZ * GYRO_SENS);
    }  
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
    seqDur = wsDurArray[0];
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
      send0(CMD_SEQUENCE_END, false);
      return;
    }
    seqDur = wsDurArray[runSequenceCount];
    seqPw = pwArray[runSequenceCount];
  }
  if (TEST_RIGHT) {
//    setPwmSpeed(MOTOR_RIGHT, seqPw);
    streamValueArray[1] = fpsRight;
  }
  if (TEST_LEFT) {
//    setPwmSpeed(MOTOR_LEFT, seqPw);
    streamValueArray[1] = fpsLeft;
  }
  if (TEST_RIGHT && TEST_LEFT) {
    streamValueArray[1] = wheelSpeedFps;
  }
  streamValueArray[2] = seqPw;
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
    seqDur = wsDurArray[0];
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
      send0(CMD_SEQUENCE_END, false);
    } 
    else { // Not done.  Load next in sequence
      seqDur = wsDurArray[runSequenceCount];
      seqWs = wsArray[runSequenceCount];
//      debugFloat("seqWs: ", seqWs);
//      debugInt("seqDur: ", seqDur);
    }
  } 
  
  setTargetSpeedRight(seqWs);    
  setTargetSpeedLeft(seqWs);

  streamValueArray[0] = timeMicroSeconds/1000;
  streamValueArray[1] = seqWs;
  streamValueArray[2] = wheelSpeedFps;
  streamValueArray[3] = gyroXAngle;
  streamValueArray[4] = accelXAngle;
  streamValueArray[5] = gaXAngle;
  streamValueArray[6] = tickDistanceRight + tickDistanceLeft;
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
//  debugOff();  // Turn off all debugging flags.
  mode = x;
  runState = STATE_RESTING;
//  debugInt("Starting mode: ", mode); // Seems to need this.
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
  case MODE_IMU:
    aImuInit();
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
  
  switch (mode) {
  case MODE_MOTOR_PW:
    break;
  case MODE_DRIVE:
    break;
  case MODE_TP4:
    aTp4Start();
    break;
  case MODE_IMU:
    aImuStart();
    break;
  case MODE_TP_SPEED:
//    resetTp();
    break;
  case MODE_TP_SEQUENCE:
//    resetTp();
    break;
  } // end switch  
}


