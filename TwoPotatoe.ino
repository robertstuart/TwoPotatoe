/*****************************************************************************-
 *                        TwoPotatoe.ino
 *              Runs on Teensy 3.6, 180 MHz
 *****************************************************************************/

#include "Ma_Hc.h"
#include "Ma_Watch.h"
#include "Ma_Up.h"
#include <SparkFunMPU9250-DMP.h>
#include <math.h>

#define XBEE_SER Serial1
#define UP_SER Serial3
#define WA_SER Serial4

// Decrease this value to get greater turn for a given angle
const float GYRO_SENS = 0.06097;      // Multiplier to get degrees.
const float ACCEL_SENSE = 1.0 / 4098.0;       // Multiplier to get force in g's.
const float IMU_R = 208.0;           // Imu samples / second

// Values for initial timeDrift???
const double PITCH_DRIFT = -27.0;
const double ROLL_DRIFT = 20.0;
const double YAW_DRIFT = 8.0;

const int IMU_INT_PIN =     17;
const int LED13_PIN = 13;

int speedTuning = 0;  // Can be 0, 1, or 2.  From the Hc.

const String TAB = "\t";

//Encoder factor
const double ENC_FACTOR = 650.0f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 650000L;  // Change pulse width to milli-fps speed

//#define TICKS_PER_PITCH_DEGREE 54.0D
#define GYRO_WEIGHT 0.9995    // Weight for gyro compared to accelerometer

// Modes of operation
const int MODE_MOTOR_CTRL  =   7;
const int MODE_PWM         =   8;
const int MODE_RUN         =   9;

unsigned int mode = MODE_RUN;
//unsigned int mode = MODE_PWM;
//unsigned int mode = MODE_MOTOR_CTRL;

// Values that can be modified to tune the system.
float valT = 1.5;   // pitch delta to fps correction
float valU = 0.98;  // cos low pass filter tc
float valV = 0.0;
float valW = 0.0;
float valX = 2.0;   // fps error to target angle
float valY = 0.2;   // angle error to fps correction
float valZ = 0.0;   // pitch offset
int intT = 0;
int intU = 0;

struct loc {
  float x;
  float y;
};

struct loc currentLoc;

bool isStartReceived = false;
int originalAction = 0;

long coTickPosition;
double startDecelSpeed = 0.0;

// System status
bool isRunReady = false;   // Reflects the Run command
bool isRunning = false;
bool isUpright = false;
bool isRunningOnGround = false;
bool isLogging = false;
bool isGettingUp = false;
bool isGettingDown = false;
bool isHcActive = false; // Hand controller connected.
bool isRouteInProgress = false;
bool isRouteStarted = false;
bool isUpRunning = false;
bool isCamPitch = false;
bool isBowlMode = false;

unsigned long gettingUpStartTime = 0UL;
unsigned long gettingDownStartTime = 0UL;

// LED patterns. When changed, picked up and sent to watchdog
int yePattern = BLINK_ON;
int buPattern = BLINK_ON;
int rePattern = BLINK_ON;
int gnPattern = BLINK_ON;

// Motor isr variables
volatile int tickPositionRight = 0;
volatile int tickPositionLeft = 0;
int tickPosition = 0;
float feetPosition = 0.0;
volatile unsigned int tickTimeRight = 0UL;  // time for the last interrupt
volatile unsigned int  tickTimeLeft = 0UL;
volatile int tickPeriodRight = 0L;     // last period. Minus for reverse.
volatile int tickPeriodLeft = 0L;

int targetMFpsRight = 0;
int targetMFpsLeft = 0;

// Wheel speed variables
float aFps = 0.0;       // Horizontal velocity from accelerometers.
float wFpsRight = 0.0f; // right feet per second
float wFpsLeft = 0.0f; // left feet per second
float wMFpsRight = 0.0f; // right milli-feet per second
float wMFpsLeft = 0.0f; // left milli-feet per second
volatile float targetWFpsRight = 0.0;
volatile float targetWFpsLeft = 0.0;
float targetWFps = 0.0;
float wFps = 0.0f;
int wMFps = 0;

// Route variables
float routeFps = 0.0;
float routeSteer = 0.0;

unsigned int statusTrigger = 0;
unsigned int oldTimeTrigger = 0;
unsigned int timeMicroseconds = 0; // Set in main loop.  Used by several routines.
unsigned int timeMilliseconds = 0; // Set in main loop from above.  Used by several routines.
unsigned int timeStart = 0;
unsigned int timeRun = 0;

int tpState = 0; // contains system state bits
int cmdState = 0;  // READY, PWR, & HOME command bits

unsigned int actualLoopTime; // Time since the last
float controllerX = 0.0; // +1.0 to -1.0 from controller
float controllerY = 0.0;  // Y value set by message from controller
char message[200] = "";   // Buffer for sprintf messages.

// IMU globals
//float gX = 0.0;
//float gZ = 0.0;
//float gPitch = 0;
//float gHoriz = 0.0;
//float gVert = 0.0;
//float speedGHoriz = 0.0;
//float distGHoriz = 0.0;
//float posGHoriz = 0.0;
//float speedGHozriz = 0.0;
int aveTotal = 0;  // Total of gyro averages taken.
float camPitch = 0.0;
float maPitch = 0.0;
float maRoll = 0.0;
float maYaw = 0.0;
float cfPitch = 0.0;
float gaPitch = 0.0;
float yAccel = 0.0;
float vAccel = 0.0;
float lpfAPitch = 0.0;
float gaRoll = 0.0;

float aAvgPitch = 0.0;
float aPitch = 0.0;
float aRoll = 0.0;

int baseFahr = 0;
int gyroPitchRaw = 0;
int gyroRollRaw = 0;
int gyroYawRaw = 0;
float gPitch = 0.0;
float gRoll = 0.0;
float gYaw = 0.0;
//float gcYaw = 0.0;
float gyroCumHeading = 0.0;
float gHeading = 0.0;
//float gcHeading = 0;
double oldLpfAPitch = 0.0;
float baseGyroTemp = 75.0;
//float gyroPitchRate;
//float oldGaPitch = 0.0;
float gyroPitchDelta = 0.0;
float timeDriftPitch = PITCH_DRIFT;

//float gyroRollRate;
//float gyroRoll = 0.0f;
//float accelRoll = 0.0f;
float timeDriftRoll = ROLL_DRIFT;


//float gyroYawRate = 0.0f;
//float gyroYawAngle = 0.0f;
//float gyroYawRawSum = 0.0;
//float temperatureDriftYaw = 0.0D;
float timeDriftYaw = YAW_DRIFT;

//int gyroErrorX = 0;
//int gyroErrorY = 0;
//int gyroErrorZ = 0;

float battVolt = 0.0; // battery
unsigned long tHc = 0UL;     // Time of last Hc packet
unsigned long timeUp = 0UL;  // Time of last packet from Up Board

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

volatile int intrMFpsRightSum = 0;
volatile int intrMFpsRightCount = 0;
volatile int intrMFpsLeftSum = 0;
volatile int intrMFpsLeftCount = 0;
int wsMFpsRight = 0;
int wsMFpsLeft = 0;
int wsFpsRight = 0;
int wsFpsLeft = 0;
double airFps = 0.0;
unsigned long airTrigger = 0L;

int ackFrameNumber = 0;
int ackFailure = 0;
int ccaFailure = 0;;
int purgeFailure = 0;

int interruptErrorsRight = 0;
int interruptErrorsLeft = 0;

unsigned int xBeeCount = 0;
int rightK = 0;
int leftK = 0;
unsigned int tr;

float tp7ControllerSpeed = 0.0;

/*****************************************************************************-
 * setup()
 *     Required by the Arduino system.  Called once at boot time.
 *****************************************************************************/
void setup() {
  XBEE_SER.begin(57600);   // XBee, See bottom of this page for settings.
  Serial.begin(115200);    // for debugging output
  UP_SER.begin(115200);    // UP Board
  WA_SER.begin(115200);    // Watchdog processor
  delay(2000);

  camInit();
  pinMode(LED13_PIN, OUTPUT);
  Serial.println("Serial & pins initialized.");
  imuInit();
  Serial.println("Navigation initialized");
  motorInit();
  Serial.println("Motors initialized.");
} // end setup()



/*****************************************************************************-
 * loop()
 *     Used only to change modes.  All algorithms should
 *     run in an infinite loop and then exit their loop
 *     whenever the mode changes.
 *****************************************************************************/
void loop() { //Main Loop
  switch (mode) {
  case MODE_RUN:
    runLoop();
    break;
  case MODE_PWM:
    pwmLoop();
    break;
  case MODE_MOTOR_CTRL:
    motorControlLoop();
    break;
  default:
    readXBee();
    break;
  } // end switch(mode)
} // End loop().



/*****************************************************************************-
 * pwmLoop()  Measure speed for pw.
 *****************************************************************************/
void pwmLoop() {
  unsigned long pwmTrigger = 0;
  int loopc = 0;
  float sum = 0.0;
  sendWaMsg(SEND_BLINK, LED_SW_GN, BLINK_FF);
  sendWaMsg(SEND_BLINK, LED_SW_RE, BLINK_FF);
  sendWaMsg(SEND_BEEP, T_UP3);
  valT = valU = 0;

  while (mode == MODE_PWM) {
    commonTasks();
    if (timeMilliseconds > pwmTrigger) {
      pwmTrigger = timeMilliseconds + 100; // 10/sec
      if (isRunning) {
        setMotorRight(abs(intT), (intT >= 0) ? true : false);
        setMotorLeft(abs(intU), (intU >= 0) ? true : false);
      } else {
        setMotorRight(0, false);
        setMotorLeft(0, false);
      }
      readSpeedRight();
      readSpeedLeft();
      sum += (wFpsRight + wFpsLeft);
      loopc++;
      if (loopc == 10) {
        Serial.print(sum/20.0); Serial.print(TAB); Serial.println(intT);
        loopc = 0;
        sum = 0.0;
      }
//      sprintf(message, "%3d   %3d  %5.2f  %5.2f", intT, intU, wFpsRight, wFpsLeft);
//      Serial.println(message);
    } // end timed loop
  } // while
} // aPwmLoop()



/*****************************************************************************-
 * motorControlLoop() Change the target fps and let
 *                    checkMotor() control the speed.
 *                    For testing
 *****************************************************************************/
void motorControlLoop() {
  unsigned int loop = 0;
  float sumRight = 0.0, sumLeft = 0.0;
  yePattern = BLINK_OFF;
  buPattern = BLINK_OFF;
  rePattern = BLINK_FF;
  gnPattern = BLINK_FF;
  setLedStates();
  sendWaMsg(SEND_BEEP, T_UP3);
  valT = valU = 0;

  while (mode == MODE_MOTOR_CTRL) {
    commonTasks();
    if (isNewImu()) {  // Use imu for timing.
      targetWFpsRight = ((float) intT) * 0.05; // Change to 0.25 fps per increment
      targetWFpsLeft = ((float) intU) * 0.05 ; //
      checkMotors();
      sumRight += wFpsRight;
      sumLeft += wFpsLeft;
      if (!(loop++ % 250)) {
        float r = sumRight / 250.0;
        float l = sumLeft / 250.0;
        sumRight = sumLeft = 0.0;
        sprintf(message, "%5.2f   %5.2f  %5.2f", l, r ,targetWFpsRight);
        Serial.println(message);
      }
    } // end timed loop
  } // while
} // motorControlLoop()


/*****************************************************************************-
 * Calibrate()  Startup routine to run the necessary routines for 1-2 sconds
 *              to calibrate the gyro and set up the starting state.
 *****************************************************************************/
//void calibrate() {
//  unsigned long stopTime = millis() + 2000;  // Bail after 2 seconds.
//  while (aveTotal < 2) { // Do it unil we have 2 1/2 second samples.
//    if (isNewImu()) {
//      imuStable();
//    }
//    if (millis() > stopTime) break;
//  }
//  oldLpfAPitch = gaPitch = aAvgPitch;  // Set gPitch to correct value at start
//}



/*********************************************************
 *
 * Xbee settings
 *
 *     The XBee 900 hz modules are left in their
 *     default state except that the baud rate is set to
 *     57k and the destination addresses are set
 *     to point to the other module.
 *
 *********************************************************/
