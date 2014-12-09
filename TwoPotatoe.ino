/* ---------------------- TwoPotatoe ----------------------- */

#include <stdlib.h>
#include "Common.h"

#define XBEE_SER Serial3
#define MINI_SER Serial1

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_LEFT_ENCA = 40;   
const int MOT_LEFT_ENCB = 42; 
const int MOT_RIGHT_PWML =  46;   
const int MOT_RIGHT_DIR =  44;   
const int MOT_RIGHT_PWMH =   2; 
const int MOT_RIGHT_ENCA =  41;  
const int MOT_RIGHT_ENCB =  43; 
const int MOT_LEFT_PWML =   51; 
const int MOT_LEFT_DIR =   45; 
const int MOT_LEFT_PWMH =    7;  

const float SPEED_MULTIPLIER = 5.0;


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

#define YE_SW_PIN 24 // Yellow switch
#define PWR_PIN 25 // Mosfet power controller
#define SPEAKER_PIN 26 // 
#define PRESSURE_PIN A0             // Pressure sensor
#define L_BATT_PIN A1              // Logic battery
#define MB_BATT_PIN A2              // Base Motor battery
#define EB_BATT_PIN A3              // Extended Motor battery

//Encoder factor
const float ENC_FACTOR = 1329.0f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 1329000L;  // Change pulse width to milli-fps speed, 1/29 gear
const float FPS_TO_TPCS = 7.52f;   // Convert foot/sec to tics/centisecond
const float ENC_BRAKE_FACTOR = ENC_FACTOR * 0.95f;

// Max int/long values
#define UNSIGNED_LONG_MAX 4294967295UL 
#define LONG_MAX  2147483647L
#define LONG_MIN -2147483648L

#define TICKS_PER_360_YAW 3030
#define TICKS_PER_180_YAW 1515
#define TICKS_PER_YAW_DEGREE 8.4166
#define TICKS_PER_PITCH_DEGREE 20.0
#define GYRO_SENS 0.009375     // Multiplier to get degree. -0.075/8?
#define DRIFT_COUNT 100
#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer

// Due has 96 kbytes sram
#define DATA_ARRAY_SIZE 1000

// Arrays to save data to be dumped in blocks.
long aArray[ DATA_ARRAY_SIZE];
short bArray[ DATA_ARRAY_SIZE];
short cArray[ DATA_ARRAY_SIZE];
short dArray[ DATA_ARRAY_SIZE];
short eArray[ DATA_ARRAY_SIZE];
short fArray[ DATA_ARRAY_SIZE];
short gArray[ DATA_ARRAY_SIZE];
int dataArrayPtr = 0;
boolean isDumpingData = false;
boolean isSerialEmpty = true;

unsigned int mode = MODE_TP5;
unsigned int oldMode = MODE_TP5;

int BEEP_UP [] = {1200, 100, 1500, 100, 0};
int BEEP_WARBLE[] = {2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 0};
int BEEP_DOWN[] = {1000, 300, 1500, 300, 0};

// new flash sequences
const byte END_MARKER = 42;
byte BLINK_OFF[] = {0,END_MARKER};  // Off
byte BLINK_SF[] = {1,0,0,0,0,0,0,0,END_MARKER};  // slow flash
byte BLINK_FF[] = {1,0,END_MARKER};  // Fast flash
byte BLINK_SB[] = {1,1,1,1,0,0,0,0,END_MARKER};  // slow blink
byte BLINK_ON[] = {1,END_MARKER};  // Off

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
  0.7,    // v rotation subtraction, 0-2.0?
  0.2,    // w cos smoothing rate.  0-1.0 **** changed from0.2 **************
  2.0,    // x CO speed error to angle factor
  0.18,   // Y Target angle to WS 
  -1.25};   // z accelerometer offset

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

int bbb = 42;

// Route
// TODO do this with an array of structures
byte actionArray[100];
int aValArray[100];
int bValArray[100];
int routeActionPtr = 0;
int routeActionSize = 0;
boolean isRouteInProgress = false;
int routeCurrentAction = 0;
float routeTargetHeading = 0.0;
long routeTargettickPosition = 0L;
long routeTargetTime = 0L;


int16_t aPitch, aRoll, aPitchRoll;
int16_t gPitch, gRoll, gYaw;
int16_t mPitch, mRoll, mYaw;
float headX, headY;
float magHeading;

// Speed and position variables
long tickPositionRight = 0L;
long tickPositionLeft = 0L;
long tpDistanceDiff = 0L;
long tickPosition;

unsigned long tickTimeRight = 0UL;  // time for the last interrupt
unsigned long tickTimeLeft = 0UL;
long tickPeriodRight = 0L;     // last period. Minus for reverse.
long tickPeriodLeft = 0L;
float targetSpeedRight = 0.0;
float targetSpeedLeft = 0.0;
long targetTickPeriodRight = 0L;
long targetMFpsRight = 0L;
long targetBrakeMFpsRight = 0L;
long targetRevMFpsRight = 0L;
long targetMFpsLeft = 0L;
long targetBrakeMFpsLeft = 0L;
long targetRevMFpsLeft = 0L;

float fpsRight = 0.0f; // right feet per second
float fpsLeft = 0.0f;  // left feet per second
float wheelSpeedFps = 0.0f;
int mWheelSpeedFps = 0;
float tickHeading = 0.0;
long tickMagCorrection = 0L;


unsigned long timeTrigger = 0L;
unsigned long statusTrigger = 0L;
unsigned long oldTimeTrigger = 0L;
unsigned long timeMicroseconds = 0L; // Set in main loop.  Used by several routines.
unsigned long timeMilliseconds = 0L; // Set in main loop from above.  Used by several routines.

int tpState = 0; // contains STATE_READY, STATE_RUNNING,STATE_UPRIGHT, STATE_ON_GROUND, STATE_MOTOR_FAULT bits
int cmdState = 0;  // READY, PWR, & HOME command bits

unsigned int pressure = 0; // pressure sensor value

unsigned int actualLoopTime; // Time since the last
float controllerX = 0.0; // +1.0 to -1.0 from controller
float controllerY = 0.0;  // Y value set by message from controller
int signalStrength = 0;

int gyroPitchRaw;  // Vertical plane parallel to wheels
float gyroPitchRate;
long gyroPitchRawSum;
int driftCount;
int driftPitch = 0;
int driftRoll = 0;
int driftYaw = 0;
float accelPitchAngle = 0.0;  // Vertical plane parallel to wheels
float gaPitch = 0.0f;
float oldGaPitch = 0.0;
float gyroPitchDelta = 0.0;
float gyroPitch = 0.0; // not needed
float accelPitch = 0.0;

int gyroRollRaw = 0;
float gyroRollRate;
float gyroRoll = 0.0f;
float accelRoll = 0.0f;
float gaRoll = 0.0f;

float gyroYawRaw = 0.0f;
float gyroYawRate = 0.0f;
float gyroYawAngle = 0.0f;
float gyroYawRawSum = 0.0;

int bmBattVolt = 0; // Base motor battery * 100
int emBattVolt = 0; // Extended motor battery * 100
int lBattVolt = 0; // Logic battery * 100
int tpDebug = 4241;

unsigned long tHc = 0L;  // Time of last Hc packet
unsigned long tPc = 0L;  // Time of last Pc packet

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

int ackMsgType = TP_RCV_MSG_NULL;
int ackMsgVal = 0;

boolean isHcCommand = false;

/* roll pitch and yaw angles computed by iecompass */
int iPhi, iThe, iPsi; 
/* magnetic field readings corrected for hard iron effects and PCB orientation */
int iBfx, iBfy, iBfz;
/* hard iron estimate */
int iVx, iVy, iVz;

float magCorrection = 0.0;

// Sequence variables
int sequenceCount = 0;
boolean sequenceIsRunning = false;
int runSequenceCount = 0;
boolean isSequence = false;
int seqDur = 0;
float seqWs = 0.0;
int seqPw = 0;
float wsArray[30];
int wsDurArray[30];
int pulseIndex = 0;
int pulseCount = 0;
int motorArray[30];
int pwArray[30];
unsigned long pulseTrigger = 0;
boolean isReceivingBlock = false;
boolean isPwData = false; // Send data after a sequence?

int actionRight = 99;
int actionLeft = 99;

int tVal = 0;
int uVal = 0;
int vVal = 0;
int wVal = 0;
int xVal = 0;
int yVal = 0;
int zVal = 0;

int mWsFpsRight = 0;
int mAverageFpsRight = 0;
int mWsFpsLeft = 0;
int mAverageFpsLeft = 0;
long mWsFpsRightSum = 0;
int mWsFpsRightCount = 0;;
long mWsFpsLeftSum = 0;
int mWsFpsLeftCount = 0;;
float airFps = 0.0;
unsigned long airTrigger = 0L;

int ackFrameNumber = 0;
int ackFailure = 0;
int ccaFailure = 0;;
int purgeFailure = 0;

float tp5LpfCosAccel = 0.0;
float tp6LpfCosAccel = 0.0;

float tp5FpsLeft = 0.0f;
float tp5FpsRight = 0.0f;
float tp6FpsLeft = 0.0f;
float tp6FpsRight = 0.0f;

int interruptErrorsRight = 0;
int interruptErrorsLeft = 0;

boolean noResendDumpData = false;
boolean isLights = false;

/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {

  // XBee, See bottom of this page for settings.
  XBEE_SER.begin(57600);  // XBee
  MINI_SER.begin(115200);  // Mini IMU processor
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
  pinMode(MB_BATT_PIN, INPUT);
  pinMode(EB_BATT_PIN, INPUT);
  pinMode(YE_SW_PIN, INPUT_PULLUP);

  digitalWrite(PWR_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(RIGHT_HL_PIN, LOW);
  digitalWrite(LEFT_HL_PIN, LOW);
  digitalWrite(REAR_BL_PIN, LOW);
  digitalWrite(SPEAKER_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  mode = MODE_TP5;
  angleInit();
  timeTrigger = micros();
  beep(BEEP_UP);
  delay(100);
  for (int i = 0; i < DATA_ARRAY_SIZE; i++) {
    aArray[i] = 42;
    bArray[i] = 9;
    cArray[i] = 4242;
    dArray[i] = i;
  }
} // end setup()



/************************************************************************** *
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
  case MODE_PWM_SPEED:
    aPwmSpeed();
    break;
  case MODE_TP_SPEED:
    aTpSpeed();
    break;
  case MODE_TP_SEQUENCE:
    aTpSequence();
    break;
  case MODE_TP5:
    aTp5Run();
    break;
  case MODE_TP6:
    aTp6Run();
    break;
  case MODE_POSITION:
    aPos();
    break;
  default:
    readXBee();
    break;
  } // end switch(mode)
} // End loop().  



/**************************************************************************.
 * aPwmSpeed()
 **************************************************************************/
void aPwmSpeed() {
  unsigned long pwmTrigger = 0;
  unsigned long tt;
  tpState = tpState | TP_STATE_RUNNING;
  motorInitTp();
//  angleInitTp7();
  setBlink(RED_LED_PIN, BLINK_SF);
  
  while (mode == MODE_PWM_SPEED) {
    commonTasks();
    while (isDumpingData) {
      dumpData();
      delay(50);
    }
    if (timeMicroseconds > pwmTrigger) {
      int action = FWD;
      pwmTrigger = timeMicroseconds + 100000; // 10/sec
      if (tVal > 0) action = FWD;
      else action = BKWD;
      setMotor(MOTOR_RIGHT, action, abs(tVal));
      if (uVal > 0) action = FWD;
      else action = BKWD;
      setMotor(MOTOR_LEFT, action, abs(uVal));
      readSpeed();      
      sendStatusFrame(XBEE_PC);
      Serial.print(interruptErrorsRight);
      Serial.print("\t"); 
      Serial.print(fpsRight); 
      Serial.print("\t"); 
      Serial.print(interruptErrorsLeft);
      Serial.print("\t"); 
      Serial.println(fpsLeft);
      interruptErrorsRight = interruptErrorsLeft = 0;
    } // end timed loop 
  } // while
} // aPwmSpeed()



/**************************************************************************.
 * aTpSpeed()
 **************************************************************************/
void aTpSpeed() {
  unsigned long tpTrigger = 0;
  tpState = tpState | TP_STATE_RUNNING;
  motorInitTp();
  setBlink(RED_LED_PIN, BLINK_SF);
  
  while (mode == MODE_TP_SPEED) {
    commonTasks();
    while (isDumpingData) {
      dumpData();
      delay(1);
    }
    if (timeMicroseconds > tpTrigger) {
      int action = FWD;
      tpTrigger = timeMicroseconds + 100000; // 10/sec
      setTargetSpeedRight(((float) tVal) / 1000.0);
      setTargetSpeedLeft(((float) uVal) / 1000.0);
      readSpeed();      
      sendStatusFrame(XBEE_PC);  // Send status message to controller.
      checkMotorRight();
      checkMotorLeft();
    } // end timed loop 
  } // while
} // aTpSpeed()



/**************************************************************************.
 * aTpSequence()
 **************************************************************************/
void aTpSequence() {
} // aTpSequence()





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
 * aPulseSequence()
 *
 *     Run the pulse sequence already set by the LOAD command.
 *
 *********************************************************/
void aPulseSequence() {
  pulseIndex = 0;
  pulseTrigger = 0L;
  
  while (mode == MODE_PULSE_SEQUENCE) {
    timeMicroseconds = micros();
    if (timeMicroseconds > pulseTrigger) {
      if (pulseIndex == pulseCount) { // End of sequence      
        mode = oldMode;
        if (isPwData) {
          isPwData = false;
          sendDumpData();
        }
      }
      else {
        int motor = motorArray[pulseIndex];
        int pw = pwArray[pulseIndex];
        pulseTrigger = timeMicroseconds + pw;
        pulseIndex++;
        switch (motor) {
        case 1: // Coast
          setMotor(MOTOR_RIGHT, COAST, 0);
          break;
        case 2: // Forward
          setMotor(MOTOR_RIGHT, FWD, 255);
          break;
        case 3: // Backwards
          setMotor(MOTOR_RIGHT, BKWD, 255);
          break;
        default: // Brake
          setMotor(MOTOR_RIGHT, BRAKE, 0);
          break;
        }
      }
    } 
  }
}
/*********************************************************
 *
 * Xbee settings
 *
 *     The XBee is configured with the default settings 
 *     except for the following:
 *
 *     CH     0x19
 *     ID     2221   (Is this needed?)
 *     MY     7770  TwoPotatoe
 *            7771  PC
 *            7772  Hand Controller
 *     MM     0     Digi mode
 *     RR     2     retries
 *     BD     7     115200 baud
 *     AP     1     API mode
 *
 *********************************************************/











