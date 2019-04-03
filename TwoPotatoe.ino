/*****************************************************************************-
 *                        TwoPotatoe
 *              Runs on Teensy 3.6, 180 MHz
 *****************************************************************************/

#include "HC.h"
#include "Watch.h"
#include "UpArd.h"
#include <Wire.h>
#include <MPU9250.h>
//#include <MPU9250_RegisterMap.h>
//#include <SparkFunMPU9250-DMP.h>

#define XBEE_SER Serial1
#define UP_SER Serial
#define WA_SER Serial4

#define TICKS_PER_FOOT 2222.0D // For Losi DB XL 1/5 scale

// Decrease this value to get greater turn for a given angle
const float GYRO_SENS = 0.0696;      // Multiplier to get degrees. 
const float ACCEL_SENS = 0.00025;       // Multiplier to get force in g's.

// Values for initial timeDrift???
const double PITCH_DRIFT = 19.0;
const double ROLL_DRIFT = 20.0;
const double YAW_DRIFT = 8.0;



const int IMU_INT_PIN =     17;

const int LED13_PIN = 13;

const float SPEED_MULTIPLIER = 20.0;
const unsigned int TP_PWM_FREQUENCY = 20000;

const int HEADING_SOURCE_G =  0;
const int HEADING_SOURCE_M =  1;
const int HEADING_SOURCE_T =  2;
const int HEADING_SOURCE_GM = 3;

#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2

#define A_LIM 20.0 // degrees at which the speedAdjustment starts reducing.
#define S_LIM 1.0  // maximum speedAdjustment;

String tab = "\t";

//Encoder factor
const double ENC_FACTOR = 650.0f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 650000L;  // Change pulse width to milli-fps speed
const double FPS_TO_TPCS = 7.52f;   // Convert foot/sec to tics/centisecond
const double ENC_BRAKE_FACTOR = ENC_FACTOR * 0.95f;

// Max int/long values
#define UNSIGNED_LONG_MAX 4294967295UL 
#define LONG_MAX  2147483647L
#define LONG_MIN -2147483648L

//#define TICKS_PER_PITCH_DEGREE 54.0D
#define GYRO_WEIGHT 0.997    // Weight for gyro compared to accelerometer
#define DEFAULT_GRID_OFFSET 0.0
#define SONAR_SENS 0.0385

//unsigned int mode = MODE_RUN;
//unsigned int mode = MODE_PWM;
unsigned int mode = MODE_MOTOR_CTRL;

// Values that can be modified to tune the system.
float valT = 4.5;
float valU = 0.98;
float valV = 2.7;
float valW = 9.5;
float valX = 2.0;
float valY = 0.2;
float valZ = -3.0;

int intT = 0;
int intU = 0;

struct loc {
  double x;
  double y;
};

struct loc currentLoc;

boolean isStartReceived = false;
//double routeTargetXYDistance = 0.0;
int originalAction = 0;
int aveTotal = 0;  // Total of gyro averages. ( <= AVE_SIZE )
double gaPitch = 0.0;
float yAccel = 0.0;
double lpfAPitch = 0.0;
double gaRoll = 0.0;

double aPitch = 0.0;
double aRoll = 0.0;

int baseFahr = 0;
int gyroPitchRaw = 0;
int gyroRollRaw = 0;
int gyroYawRaw = 0;
double gPitch = 0.0;
double gRoll = 0.0;
double gYaw = 0.0;
double gcYaw = 0.0;
//double gyroCumHeading = 0.0;
double gHeading = 0;
double gcHeading = 0;

double rotation2 = 0.0D;
double cos2 = 0.0D;
double lpfCos2 = 0.0D;
double lpfCosOld2 = 0.0D;
float rotation3 = 0.0;
float cos3 = 0.0;
float lpfCos3 = 0.0;
float lpfCos3Old = 0.0;
float lpfCos3Accel = 0.0;

long coTickPosition;
double startDecelSpeed = 0.0;

// System status 
boolean isRunReady = false;   // Reflects the Run command
boolean isRunning = false;
boolean isUpright = false;

boolean isHcActive = false; // Hand controller connected.

// Motor isr variables
volatile int tickPositionRight = 0;
volatile int tickPositionLeft = 0;
int tickPosition = 0;
volatile unsigned int tickTimeRight = 0UL;  // time for the last interrupt
volatile unsigned int  tickTimeLeft = 0UL;
volatile int tickPeriodRight = 0L;     // last period. Minus for reverse.
volatile int tickPeriodLeft = 0L;

int targetMFpsRight = 0;
int targetMFpsLeft = 0;

// Wheel speed variables
float wFpsRight = 0.0f; // right feet per second
float wFpsLeft = 0.0f; // left feet per second
float wMFpsRight = 0.0f; // right milli-feet per second
float wMFpsLeft = 0.0f; // left milli-feet per second
volatile float targetWFpsRight = 0.0;
volatile float targetWFpsLeft = 0.0;
float targetWFps = 0.0;
float wFps = 0.0f;
int wMFps = 0;

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
char message[100] = "";
char message1[80] = "message 1";
char message2[80] = "message 2";
char message3[80] = "message 3";
char message4[80] = "message 4";

// IMU globals
float baseGyroTemp = 75.0;
float gyroPitchRate;
float oldGaPitch = 0.0;
float gyroPitchDelta = 0.0;
float timeDriftPitch = PITCH_DRIFT;

float gyroRollRate;
float gyroRoll = 0.0f;
float accelRoll = 0.0f;
float timeDriftRoll = ROLL_DRIFT;


float gyroYawRate = 0.0f;
float gyroYawAngle = 0.0f;
float gyroYawRawSum = 0.0;
float temperatureDriftYaw = 0.0D;
float timeDriftYaw = YAW_DRIFT;

int gyroErrorX = 0;
int gyroErrorY = 0;
int gyroErrorZ = 0;

float battVolt = 0; // battery 
volatile int debugInt1 = 42;
volatile int debugInt2 = 42;
volatile int debugInt3 = 42;
volatile int debugInt4 = 42;
volatile float debugFloat1 = 42.42;
volatile float debugFloat2 = 42.42;
volatile float debugFloat3 = 42.42;
volatile float debugFloat4 = 42.42;
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
//  while(!Serial) {}  // Wait for serial to start up

  pinMode(LED13_PIN, OUTPUT);
  Serial.println("Serial & pins initialized.");
  imuInit();
  Serial.println("Navigation initialized");
  motorInit();
  Serial.println("Motors initialized.");
  calibrate();
  sendWaMsg(SEND_BEEP, MUSIC_UP2);
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
  sendWaMsg(SEND_BEEP, MUSIC_UP3);
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
        Serial.print(sum/20.0); Serial.print(tab); Serial.println(intT);
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
 *****************************************************************************/
void motorControlLoop() {
  unsigned int loop = 0;
  float sumRight = 0.0, sumLeft = 0.0;
  sendWaMsg(SEND_BLINK, LED_SW_GN, BLINK_FF);
  sendWaMsg(SEND_BLINK, LED_SW_RE, BLINK_FF);
  sendWaMsg(SEND_BEEP, MUSIC_UP3);
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
void calibrate() {
  unsigned long stopTime = millis() + 2000;  // Bail after 2 seconds.
  while (aveTotal < 2) { // Do it unil we have 2 1/2 second samples.
    if (isNewImu()) {
      doGyroDrift();
    }
    if (millis() > stopTime) break;
  }
  gPitch = aPitch;  // Set gPitch to correct value at start
  sendWaMsg(SEND_BLINK, LED_SW_GN, BLINK_OFF);  // Turn all the leds off.
  sendWaMsg(SEND_BLINK, LED_SW_RE, BLINK_OFF);
  sendWaMsg(SEND_BLINK, LED_SW_BU, BLINK_OFF);
  sendWaMsg(SEND_BLINK, LED_SW_YE, BLINK_SF); // Set to startup state
}



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
