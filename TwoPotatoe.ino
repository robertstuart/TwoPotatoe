/* ---------------------- TwoPotatoe ---------------------- */

#include <stdlib.h>
#include "Common.h"

#define MYSER Serial3
const byte RESET[] = {0xFE, 0x00, 0x04, 0x08, 0x0, 0x46, 0x52, 0x5F};
unsigned int mode = MODE_TP4;

// set motor to test
const boolean TEST_RIGHT = true;
const boolean TEST_LEFT = true;

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_RIGHT_ENCA = 41;   
const int MOT_RIGHT_ENCB = 43; 
const int MOT_RIGHT_INA =  45;   
const int MOT_RIGHT_INB =  47;   
const int MOT_RIGHT_SF =   49;    
const int MOT_RIGHT_PWM =   2; 
const int MOT_LEFT_ENCA =  40;  
const int MOT_LEFT_ENCB =  42; 
const int MOT_LEFT_INA =   44; 
const int MOT_LEFT_INB =   46; 
const int MOT_LEFT_SF =    48; 
const int MOT_LEFT_PWM =    3;  

// State machine for messaging between TP, PC, & HC.
const int MSG_STATE_TIMER_WAIT = 0;
const int MSG_STATE_PC_ACK = 2;
const int MSG_STATE_HC_ACK = 3;
const int MSG_STATE_PC_NAK = 4;
const int MSG_STATE_HC_NAK = 5;

const int ACK_PC = 100; // Message frames >= this have been sent to PC


const float SPEED_MULTIPLIER = 3.0;


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
#define RIGHT_HL_PIN 10 // headlamp
#define LEFT_HL_PIN 9 // headlamp
#define REAR_BL_PIN 8 // rear lamp

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
#define TICKS_PER_360 3030
#define TICKS_PER_180 1515
#define TICKS_PER_DEGREE 8.4166
//#define TICKS_PER_180 1515

// Constants for IMU
#define GYRO_SENS 0.009375     // Multiplier to get degree. -0.075/8?
//#define INVEN_GYRO_SENS 0.009375     // Multiplier to get degree. -0.075/8?
#define DRIFT_COUNT 100
#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer
#define TICK_WEIGHT 0.98    // Weight for tick compared to accelerometer
#define TICKS_PER_FOOT 800L
#define TICKS_PER_TFOOT 80L
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
#define MOTOR_PULSE_LENGTH 1500L

#define MAX_PULSE_SPEED 0.8

#define END_MARKER 42
//#define MAX_PULSE_WAIT 8000
//#define PULSE_LENGTH 1500
int BEEP_UP [] = {1200, 100, 1500, 100, 0};
int BEEP_WARBLE[] = {1200, 300, 1500, 300, 
                  1200, 300, 1500, 300, 
                  1200, 300, 1500, 300, 
                  1200, 300, 1500, 300, 0};
int BEEP_DOWN[] = {1000, 300, 1500, 300, 0};
// bit0=blue, bin1= yellow, bit2=red
// each state lasts 1/10 second
byte BLINK_R_FB[] = {
  1,4,4,4,4,4,4,4,END_MARKER};  // red on, flash blue
byte BLINK_B_FY[] = {
  2,1,1,1,1,1,1,1,END_MARKER};  // blue on, flash yellow
byte BLINK_FRG[] = {
  4,1,END_MARKER};               // rapid red-green
byte BLINK_F_RG[] = {
  4,4,4,4,1,END_MARKER};               // red, flash green
byte BLINK_SBYG[] = {
  7,7,7,7,0,0,0,0,END_MARKER};  // blink all slow flash
byte BLINK_FY[] = {
  2,0,END_MARKER};                // yellow flash
byte BLINK_FR[] = {
  4,0,END_MARKER};                // red flash
byte BLINK_FB[] = {
  1,0,END_MARKER};                // blue flash
byte BLINK_SB[] = {
  1,1,1,1,1,1,1,1,0,0,0,0,END_MARKER};    // blue slow

typedef struct {
  float t;
  float u;
  float v;
  float w;
  float x;
  float y;
  float z;
} 
valSet;

valSet tp4A = { 
  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
  0.0,    // u tick angle added in.  ~0-2.0. 
  1.4,    // v rotation subtraction, 0-2.0?
  0.2,    // w cos smoothing rate.  0-1.0 **** changed from0.2 **************
  2.0,    // x CO speed error to angle factor
  0.18,   // Y Target angle to WS 
  -2.7}; // z accelerometer offset

valSet tp4B = { 
  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
  0.0,    // u tick angle added in.  ~0-2.0. 
 1.13,    // v rotation subtraction, 0-2.0?
  0.3,    // w cos smoothing rate.  0-1.0
  3.0,    // x CO speed error to angle factor
  0.15,   // Y Target angle to WS
  -2.7}; // z accelerometer offset

valSet tp4C = { 
  0.5,    // t tick angle decay rate. zero = rapid decay rate, 1 = none.
  0.0,    // u tick angle added in.  ~0-2.0. 
 1.13,    // v rotation subtraction, 0-2.0?
  0.3,    // w cos smoothing rate.  0-1.0
  3.0,    // x CO speed error to angle factor
  0.15,   // Y Target angle to WS
  -2.7}; // z accelerometer offset

valSet *currentValSet = &tp4A;
int vSetStatus = VAL_SET_A;

// TODO do this with an array of structures
byte actionArray[100];
int aValArray[100];
int bValArray[100];
int routeActionPtr = 0;
int routeActionSize = 0;
boolean isRouteInProgress = false;
boolean isBlockInProgress = false;
int routeCurrentAction = 0;
float routeTargetHeading = 0.0;
long routeTargetTickDistance = 0L;
long routeTargetTime = 0L;

int msgState = MSG_STATE_TIMER_WAIT;

int aPitch, aRoll, aYaw;
int gPitch, gRoll, gYaw;
int mPitch, mRoll, mYaw;
float mPitchVec, mRollVec, mYawVec;
float headX, headY;
float magHeading;

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
float wheelSpeedFps = 0.0f;
float speedTpcs = 0.0f;
unsigned long lasttime, gap;
float tickHeading = 0.0;
long tickMagCorrection = 0L;

unsigned long waitPeriodRight = 0UL;  // Wait beyond beginning of pulse!!!
unsigned long waitPeriodLeft = 0UL;  // Wait beyond beginning of pulse!!!

int targetDirectionRight = FWD;
int targetDirectionLeft = FWD;

//long streamValueArray[10] = {
//  0,0,0,0,0,0,0,0,0,0};

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

unsigned long timeTrigger = 0L;
unsigned long oldTimeTrigger = 0L;
unsigned long timeMicroseconds = 0L; // Set in main loop.  Used by several routines.
unsigned long timeMilliseconds = 0L; // Set in main loop from above.  Used by several routines.

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

int tpState = 0; // contains STATE_READY, STATE_RUNNING,STATE_UPRIGHT, STATE_ON_GROUND, STATE_MOTOR_FAULT bits
int cmdState = 0;  // READY, PWR, & HOME command bits

long home = 0L;  // home tpPosition, zero indicates no home position.
unsigned int pressure = 0; // pressure sensor value
unsigned int tpPw = 2000;

unsigned int actualLoopTime; // Time since the last
float controllerX = 0.0; // +1.0 to -1.0 from controller
float controllerY = 0.0;  // Y value set by message from controller
int signalStrength = 0;

int gyroPitchRaw;  // Vertical plane parallel to wheels
float gyroPitchRate;
float gyroPitchAngle = 0.0;
float gyroPitchAngleDelta = 0.0;
int gyroPitchDriftCount = 0;
long gyroPitchRawSum;
int driftCount;
int driftPitch = 0;
int driftRoll = 0;
int driftYaw = 0;
float accelPitchAngle = 0.0;  // Vertical plane parallel to wheels
float gaPitchAngle = 0.0f;

// TP5 angle variables
float gaPitchTickAngle = 0.0f;
float oldDeltaOverBase = 0.0;
int deltaStorePtr = 0;
long oldTp5TickDistance = 0L;
float tp5IntTickRate = 0.0;
float deltaSum = 0.0;
float deltaOverBase = 0.0;
int tp5TickRate = 0;

int gyroRollRaw = 0;
float gyroRollRate;
float gyroRollAngle = 0.0f;
float accelRollAngle = 0.0f;
float gaRollAngle = 0.0f;

float gyroYawRaw = 0.0f;
float gyroYawRate = 0.0f;
float gyroYawAngle = 0.0f;
float gyroYawRawSum = 0.0;

long oldTickRate = 0;
float oldtpAngle = 0.0;
float tpAngle1Rate = 0.0;
boolean straightMode = false;

float coDistance = 0.0f;
float tpAngle2 = 0.0f;
float tpAngle2Rate = 0.0f;

float rotateTarget = 0.0;
boolean isRotating = false;
//
unsigned int byteCountErrorsHc = 0;
int txRateDivider = 5;
boolean txRateHL = false;
int txRateCounter = 0;

int batteryVolt = 0;
float gyroXAngleDelta = 0;

long oldTickDistance = 0;
int oldTickSpeed = 0;
float oldTickAngle = 0.0f;

int unknownCmdErrors = 0;

int lamp;
unsigned long tHc = 0L;
unsigned long tPc = 0L;

const int MAX_PACKET_SIZE = 100;

byte sendArray[MAX_PACKET_SIZE + 1];
unsigned int packetByteCount = 0;
unsigned int dataPtr = 0;
int packetValue;   // int value
boolean isPacketInProgress = false;
unsigned int packetSource;
unsigned int packetSignal;
boolean isTxStatusMessage = false;
int txAckFrame = 0;

int debugVal = NO_DEBUG;
int ackMsgType = TP_RCV_MSG_NULL;
int ackMsgVal = 0;

boolean isHcCommand = false;

const byte* blinkPattern = BLINK_FY;
int blinkPatternSize = sizeof(BLINK_FY);
int blinkPtr = 0;

/* roll pitch and yaw angles computed by iecompass */
int iPhi, iThe, iPsi; 
/* magnetic field readings corrected for hard iron effects and PCB orientation */
int iBfx, iBfy, iBfz;
/* hard iron estimate */
int iVx, iVy, iVz;

float magCorrection = 0.0;

// Values for TP6
long ttdR = 0;
long ttdL = 0;
long targetTDR = 0;
long targetTDL = 0;
long loopTickDistanceR = 0;
long loopTickDistanceL = 0;
long tp5LoopTimeR = 0;
long tp5LoopTimeL = 0;
long tp6LoopTimeR = 0L;
long tp6LoopTimeL = 0L;
long fpsRightLong = 0L;
long fpsLeftLong = 0L;
unsigned long tp5LoopTime = 0;

/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {

  //  MYSER.begin(115200);
  // XBee, See bottom of this page for settings.
  MYSER.begin(57600);  // 113000 Rate matched by testing.
  Serial.begin(115200); // for debugging output
  //  resetXBee();
  pinMode(LED_PIN,OUTPUT);  // Status LED, also blue LED
  pinMode(YELLOW_LED_PIN,OUTPUT);
  pinMode(RED_LED_PIN,OUTPUT);
  pinMode(RIGHT_HL_PIN, OUTPUT);
  pinMode(LEFT_HL_PIN, OUTPUT);
  pinMode(REAR_BL_PIN, OUTPUT);
  pinMode(PWR_PIN,OUTPUT);  // Power mosfet control
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(YE_SW_PIN, INPUT_PULLUP);

  digitalWrite(PWR_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(RIGHT_HL_PIN, LOW);
  digitalWrite(LEFT_HL_PIN, LOW);
  digitalWrite(REAR_BL_PIN, LOW);
  digitalWrite(SPEAKER_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  angleInit();
  timeTrigger = micros();
  beep(BEEP_UP);
  delay(100);
  
  // Start in PWM mode if the yellow button is down.
  if (digitalRead(YE_SW_PIN) == LOW) {
    mode = MODE_PWM_SPEED;
  } 
  else {
    mode = MODE_TP5;
  }
} // end setup()



/*********************************************************
 *
 * loop()
 *
 *     Used only to change modes.  All algorithms should
 *     run in an infinite loop and then exit their loop
 *     whenever the mode changes.
 *
 *********************************************************/
void loop() { //Main Loop
  switch (mode) {
  case MODE_TP6:
    aTp6Run();
    break;
  case MODE_TP5:
    aTp5Run();
    break;
  case MODE_IMU:
    aImuRun();
    break;
  case MODE_PWM_SPEED:
    aPwmSpeedRun();
    break;
  case MODE_TP_SPEED:
    aTpSpeedRun();
    break;
  default:
    readXBee();
    break;
  } // end switch(mode)
} // End loop().  



/************************************************************************
 * aImuRun()
 *     Testing the IMU.
 * 
 * 
 ************************************************************************/
void aImuRun() {
  short accelX = 0;
  int velX = 0;
  int distX = 0;
  short accelY = 0;
  int velY = 0;
  int distY = 0;
  short accelZ = 0;
  int16_t ax, ay, az;

  timeMicroseconds = timeTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  //  imu9150.initialize();
  setBlink(BLINK_SB);

  while (mode == MODE_IMU) {
//    imu9150.getAcceleration(&ax, &ay, &az);
    if ((ax != accelX) || (ay != accelY) || (az != accelZ)) { // if any values are new.
      accelX = ax;
      velX = velX + accelX;  // Velocity = sum of accelerations at sample rate.
      distX = distX + velX;   // Distance = sum of velocities at sample rate.  
      accelY = ay;
      velY = velY + accelY;  // Velocity = sum of accelerations at sample rate.
      distY = distY + velY;   // Distance = sum of velocities at sample rate.  
      accelZ = az;
    }

    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if(timeMicroseconds > timeTrigger) {  // Loop executed every XX microseconds 
      timeTrigger += 10000;  // 100 per second

      readXBee();  // Read commands from PC or Hand Controller
      led();
      battery();

//      streamValueArray[0] = accelX;
//      streamValueArray[1] = velX;
//      streamValueArray[2] = distX;
//      streamValueArray[3] = accelY;
//      streamValueArray[4] = velY;
//      streamValueArray[5] = distY;
//      monitorImu();
    }
  }
}


/*********************************************************
 *
 * aPwmSpeedRun()
 *
 *     Run the MODE_PWM_SPEED algorithm
 *
 *********************************************************/
void aPwmSpeedRun() { 
  txRateDivider = 5;  // 20/sec
  timeTrigger = timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  motorInitPwm();
  setPwmSpeed(MOTOR_RIGHT, 0);
  setPwmSpeed(MOTOR_LEFT, 0);
  setBlink(BLINK_FY);

  while(mode == MODE_PWM_SPEED) { // main loop
    readXBee();  // Read commands from PC or Hand Controller
    led();
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    flushSerial();

    // Timed loop
    if(timeMicroseconds > timeTrigger) {  // Loop executed every XX microseconds 
      timeTrigger += 100000;  // 10 per second

      readSpeed();
      battery(); 
      if ((tpState & TP_STATE_RUN_READY) == 0) {
        tpState = tpState & (~TP_STATE_RUNNING); // unset the bit
      }
      else {
        tpState = tpState | TP_STATE_RUNNING; // set the bit
      }
      setPwmSpeed(MOTOR_RIGHT, remotePwR);
      setPwmSpeed(MOTOR_LEFT, remotePwL);

      // Fill out the sendArray and send it.
      set4Byte(sendArray, TP_SEND_A_VAL, tickDistanceRight);
      set4Byte(sendArray, TP_SEND_B_VAL, tickDistanceLeft);
      int right = (int) (fpsRight * 100.0);
      set2Byte(sendArray, TP_SEND_C_VAL, right);
      int left = (int) (fpsLeft * 100.0);
      set2Byte(sendArray, TP_SEND_D_VAL, left);
      sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_E_VAL);  
    } // End if(time) 
  } // End while(mode)
} // End aPwmSpeedRun()

/*********************************************************
 *
 * aTpSpeedRun()
 *
 *     Run the MODE_TP_SPEED algorithm
 *
 *********************************************************/
void aTpSpeedRun() { 
  txRateDivider = 5;  // 20/sec
  timeTrigger = timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  motorInitTp();
  setTargetSpeedRight(0.0f);
  setTargetSpeedLeft(0.0f);
  setBlink(BLINK_FB);

  while(mode == MODE_TP_SPEED) { // main loop
    readXBee();  // Read commands from PC or Hand Controller
    led();
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    flushSerial();
    checkMotorRight();
    checkMotorLeft();

    // Do the timed loop
    if(timeMicroseconds > timeTrigger) {  
      timeTrigger += 10000;  // 100 per second

      // Set the RUNNING bit if the READY bit is set.
      if ((tpState & TP_STATE_RUN_READY) == 0) {
        tpState = tpState & (~TP_STATE_RUNNING); // unset the bit
      }
      else {
          tpState = tpState | TP_STATE_RUNNING; // set the bit
      }
      setTargetSpeedRight(controllerY * 6.0);
      setTargetSpeedLeft(controllerY * 6.0);
      readSpeed();      
      battery();
      
      // Send the packet
      set4Byte(sendArray, TP_SEND_A_VAL, tickDistanceRight);
      set4Byte(sendArray, TP_SEND_B_VAL, tickDistanceLeft);
      int right = (int) (fpsRight * 100.0);
      set2Byte(sendArray, TP_SEND_C_VAL, right);
      int left = (int) (fpsLeft * 100.0);
      set2Byte(sendArray, TP_SEND_D_VAL, left);
      sendTXFrame(XBEE_BROADCAST, sendArray, TP_SEND_E_VAL);  
    }
  }
}


/*********************************************************
 *
 * checkDrift()
 *
 *     read gyro for DRIFT_COUNT centiseconds to cancel out drift;
 *
 *********************************************************/
void checkDrift() {
  if (driftCount > 0) {

    gyroYawRawSum += gyroYawRaw;
    driftCount--;
    if (driftCount == 0) {
      driftPitch = gyroPitchRawSum / DRIFT_COUNT;
      driftYaw = gyroYawRawSum / DRIFT_COUNT;
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
//      send0(CMD_SEQUENCE_END, false);
      return;
    }
    seqDur = wsDurArray[runSequenceCount];
    seqPw = pwArray[runSequenceCount];
  }
  if (TEST_RIGHT) {
    //    setPwmSpeed(MOTOR_RIGHT, seqPw);
//    streamValueArray[1] = fpsRight;
  }
  if (TEST_LEFT) {
    //    setPwmSpeed(MOTOR_LEFT, seqPw);
//    streamValueArray[1] = fpsLeft;
  }
  if (TEST_RIGHT && TEST_LEFT) {
//    streamValueArray[1] = wheelSpeedFps;
  }
//  streamValueArray[2] = seqPw;
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
  gyroPitchAngle = 0.0f;
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
//      send0(CMD_SEQUENCE_END, false);
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

//  streamValueArray[0] = timeMicroseconds/1000;
//  streamValueArray[1] = seqWs;
//  streamValueArray[2] = wheelSpeedFps;
//  streamValueArray[3] = gyroXAngle;
//  streamValueArray[4] = accelXAngle;
//  streamValueArray[5] = gaXAngle;
//  streamValueArray[6] = tickDistanceRight + tickDistanceLeft;
}
/*********************************************************
 *
 * Xbee settings
 *
 *     The XBee is configured with the default settings 
 *     except for the following:
 *
 *     CH     19
 *     IC     2221   (Is this needed?)
 *     MY     7770  TwoPotatoe
 *            7771  PC
 *            7772  Hand Controller
 *     BD     7     115200 baud
 *     AP     1     API mode
 *
 *********************************************************/











