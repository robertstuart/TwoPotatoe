/* ---------------------- TwoPotatoe ----------------------- */

#include "Common.h"
#include "pwm01.h"
//#include <DueTimer.h>
#include <Wire.h>
//#include <Wire1.h>
#include <L3G.h>
#include <LSM303.h>

#define XBEE_SER Serial3
#define BLUE_SER Serial1

//#define TICKS_PER_FOOT 3017.0D  // For G-made inflatable
//#define TICKS_PER_FOOT 3342.0D // For Pro-Line Masher 2.8" PRO1192-12
#define TICKS_PER_FOOT 2222.0D // For Losi DB XL 1/5 scale
#define TICKS_PER_CIRCLE_YAW  11900.0  // Larger number increases degrees turn

L3G gyro;
LSM303 compass;

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_RIGHT_ENCA =  22;  
const int MOT_LEFT_ENCA =   23;   
const int MOT_RIGHT_ENCB =  24; 
const int MOT_LEFT_ENCB =   25; 
const int MOT_RIGHT_ENCZ =  26; 
const int MOT_LEFT_ENCZ =   27; 

const int MOT_RIGHT_MODE =  50;   
const int MOT_LEFT_MODE =   51; 
const int MOT_RIGHT_DIR =    4;   
const int MOT_LEFT_DIR =     5 ; 
const int MOT_RIGHT_PWMH =   6; 
const int MOT_LEFT_PWMH =    7;  

const double SPEED_MULTIPLIER = 12.0;
const unsigned int TP_PWM_FREQUENCY = 10000;

const int HEADING_SOURCE_G =  0;
const int HEADING_SOURCE_M =  1;
const int HEADING_SOURCE_T =  2;
const int HEADING_SOURCE_GM = 3;

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
#define REAR_TL_PIN 8 // rear lamp
#define GREEN_LED_PIN 38 // LED in switch

#define BU_SW_PIN 39 // Blue switch
#define YE_SW_PIN 34 // Yellow switch
#define RE_SW_PIN 37 // Red switch
#define GN_SW_PIN 35 // Green switch

#define SPEAKER_PIN 36 // 
#define BATT_PIN A0
#define R_FORCE_PIN A1             // Force sensor
#define L_FORCE_PIN A2             // Force sensor
#define R_CURRENT_PIN A3             // Right motor current
#define L_CURRENT_PIN A4             // Left motor current

#define SONAR_RIGHT_AN A3
#define SONAR_FRONT_AN A4
#define SONAR_LEFT_AN A5
#define SONAR_RIGHT_RX 41
#define SONAR_FRONT_RX 43
#define SONAR_LEFT_RX 45

#define A_LIM 10.0 // degrees at which the speedAdjustment starts reducing.
#define S_LIM 1.0  // maximum speedAdjustment;

//Encoder factor
//const double ENC_FACTOR = 1329.0f;  // Change pulse width to fps speed, 1/29 gear
//const long ENC_FACTOR_M = 1329000L;  // Change pulse width to milli-fps speed, 1/29 gear
const double ENC_FACTOR = 650.0f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 650000L;  // Change pulse width to milli-fps speed, 1/29 gear
const double FPS_TO_TPCS = 7.52f;   // Convert foot/sec to tics/centisecond
const double ENC_BRAKE_FACTOR = ENC_FACTOR * 0.95f;

// Max int/long values
#define UNSIGNED_LONG_MAX 4294967295UL 
#define LONG_MAX  2147483647L
#define LONG_MIN -2147483648L

//#define TICKS_PER_FOOT 1536.0D
//#define TICKS_PER_RADIAN_YAW (TICKS_PER_CIRCLE_YAW / TWO_PI)
#define TICKS_PER_DEGREE_YAW (TICKS_PER_CIRCLE_YAW / 360.0)
//#define TICKS_PER_PITCH_DEGREE 20.0
#define TICKS_PER_PITCH_DEGREE 54.0D
#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer
#define DEFAULT_MAP_ORIENTATION 0.0
#define SONAR_SENS 0.0385

// Decrease this value to get greater turn for a given angle
//#define GYRO_SENS 0.009375     // Multiplier to get degree. -0.075/8?
//#define GYRO_SENS 0.0085     // Multiplier to get degree. -0.075/8?
#define GYRO_SENS 0.00834139     // Multiplier to get degree. subtract 1.8662%

#define INVALID_VAL -123456.78D

// Due has 96 kbytes sram
#define DATA_ARRAY_SIZE 2500

// Arrays to save data to be dumped in blocks.
long  aArray[ DATA_ARRAY_SIZE];
short bArray[ DATA_ARRAY_SIZE];
short cArray[ DATA_ARRAY_SIZE];
short dArray[ DATA_ARRAY_SIZE];
short eArray[ DATA_ARRAY_SIZE];
short fArray[ DATA_ARRAY_SIZE];
short gArray[ DATA_ARRAY_SIZE];
int dataArrayPtr = 0;
boolean isSerialEmpty = true;

unsigned int mode = MODE_TP6;
boolean motorMode = MM_DRIVE_BRAKE;  // Can also be MM_DRIVE_COAST
//unsigned int mode = MODE_TP_SPEED;
unsigned int oldMode = MODE_TP5;  // Used by PULSE_SEQUENCE

int BEEP_UP [] = {1200, 100, 1500, 100, 0};
int BEEP_WARBLE[] = {2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 0};
int BEEP_DOWN[] = {1000, 300, 1500, 300, 0};

// new flash sequences
const byte END_MARKER = 42;
byte BLINK_OFF[] = {0,END_MARKER};               // Off
byte BLINK_SF[] = {1,0,0,0,0,0,0,0,END_MARKER};  // Slow flash
byte BLINK_FF[] = {1,0,END_MARKER};              // Fast flash
byte BLINK_SB[] = {1,1,1,1,0,0,0,0,END_MARKER};  // Slow blink
byte BLINK_ON[] = {1,END_MARKER};                // On

struct valSet {
  double t;
  double u;
  double v;
  double w;
  double x;
  double y;
  double z;
};

struct valSet tp4A = { 
  0.5,    // t 
  0.0,    // u 
  0.7,    // v
  0.2,    // w 
  2.0,    // x 
  0.18,   // Y 
  3.3};   // z 

struct valSet tp4B = { 
  0.5,    // t 
  0.0,    // u 
 1.13,    // v 
  0.3,    // w 
  3.0,    // x 
  0.15,   // Y 
  3.3}; // z 

struct valSet tp4C = { 
  0.5,    // t 
  0.0,    // u 
 1.13,    // v 
  0.3,    // w 
  3.0,    // x 
  0.15,   // Y 
  3.3}; // z 

struct valSet tp6 = { 
  4.8,    // t
  0.1,    // u
  4.0,    // v - was 2.0, set to 3.0 for low speed
  0.18,    // w
  0.05,    // x
 130.0,   // y
  -1.2}; // z accelerometer offset

valSet *currentValSet = &tp6;
int vSetStatus = VAL_SET_A;
int bbb = 42;
 

struct loc {
  double x;
  double y;
};

struct loc currentMapLoc;
struct loc routeTargetLoc;
struct loc phantomTargetLoc;
struct loc currentAccelSelfLoc;
struct loc currentAccelMapLoc;

boolean isAngleControl = false;
boolean decelStopped = false;

int routeStepPtr = 0;
String routeTitle = "No route";

  int routeStartTickTurn = 0;
  int turnTickProgress = 0;
  double turnTargetBearing = 0.0;
  double turnTrim = 0.0;

char routeCurrentAction = 0;
double routeTargetBearing = 0.0;
double phantomTargetBearing = 0.0;
double routeMagTargetBearing = 0.0;
boolean isReachedMagHeading = false;
boolean routeIsRightTurn = true;
long routeTargetTickPosition = 0L;
double routeFps = 0.0;
double routeRadius = 0.0;
int routeWaitTime = 0L;
boolean isEsReceived = false;
boolean isRouteTargetIncreasing = false;
double routeTargetXY = 0.0;
double mapOrientation = 0.0;
long navOldTickPosition = 0L;
double currentMapHeading = 0.0;
double routeTargetXYDistance = 0.0;
double routeCoDistanceXY = 0.0;
double routeSonarMin = 0.0;
double routeSonarMax = 0.0;
double routeSonarDist = 0.0;
double routeCoDistance = 0.0;
int originalAction = 0;

int gyroTempCompX = 200;
int gyroTempCompY = 0;
int gyroTempCompZ = 105;
int sumX = 0;
int sumY = 0;
int sumZ = 0;
int meanX = 0;
int meanY = 0;
int meanZ = 0;

double gaPitch = 0.0;
double gaFullPitch = 0.0;
double tgaPitch = 0.0;
double gaRoll = 0.0;

double aPitch = 0.0;
double aRoll = 0.0;

int pitchDrift = 0;
int rollDrift = 0;
int yawDrift = 0;
double gPitch = 0.0;
double gRoll = 0.0;
double gYaw = 0.0;
double gyroCumHeading = 0.0;
double gyroHeading = 0;

double tPitch = 0.0D;
double oldTPitch = 0.0D;
double tgPitch = 0.0D;
double tgPitchDelta = 0.0D;
double oldTgPitch = 0.0D;

double accelFpsSelfX = 0.0;
double accelFpsSelfY = 0.0;
double accelFpsMapX = 0.0;
double accelFpsMapY = 0.0;

double rotation2 = 0.0D;
double cos2 = 0.0D;
double lpfCos2 = 0.0D;
double lpfCosOld2 = 0.0D;
double oldGyroCumHeading = 0.0D;
double oldTickCumHeading = 0.0D;

double headX, headY;

int16_t mX, mY, mZ;

double magHeading = 0.0; // In degrees.
double magCumHeading = 0.0;
double magRotations = 0.0;
double tickHeading = 0.0; // In degrees.
double tickCumHeading = 0.0; // In degrees.
int tickHeadingOffset = 0;
double tmHeading = 0.0;
double tmCumHeading = 0.0;
double gmHeading = 0.0;
double gmCumHeading = 0.0;
double currentX = 0.0;
double currentY = 0.0;
int fixPosition = 0;
double fixHeading = 0.0;

// Speed and position variables
long tickPositionRight = 0L;
long tickPositionLeft = 0L;
long tpDistanceDiff = 0L;
long tickPosition;
long coTickPosition;
double tp6LpfCos = 0.0;
double startDecelSpeed = 0.0;

// System status (used to be status bits
boolean isRunReady = false;   // Reflects the Run command
boolean isRunning = false;
boolean isUpright = false;
boolean isLifted = true;
boolean isOnGround = false;
boolean isHcActive = false; // Hand controller connected.
boolean isPcActive = false; // PC connected
boolean isRouteInProgress  = false; // Route in progress
boolean isDumpingData = false; // Dumping data
boolean isHoldHeading = false; // 
boolean isSpin = false; // 
boolean isSpinExceed = false; // Spinning exeeds max rate?
boolean isStand = false; // 

int standTPRight = 0;
int standTPLeft = 0;

unsigned long tickTimeRight = 0UL;  // time for the last interrupt
unsigned long tickTimeLeft = 0UL;
long tickPeriodRight = 0L;     // last period. Minus for reverse.
long tickPeriodLeft = 0L;
double targetSpeedRight = 0.0;
double targetSpeedLeft = 0.0;
long targetTickPeriodRight = 0L;
long targetMFpsRight = 0L;
long targetBrakeMFpsRight = 0L;
long targetRevMFpsRight = 0L;
long targetMFpsLeft = 0L;
long targetBrakeMFpsLeft = 0L;
long targetRevMFpsLeft = 0L;

double fpsRight = 0.0f; // right feet per second
double fpsLeft = 0.0f;  // left feet per second TODO rename!
double wheelSpeedFps = 0.0f;
int mWheelSpeedFps = 0;

unsigned long gyroTrigger = 0UL;
unsigned long statusTrigger = 0UL;
unsigned long oldTimeTrigger = 0UL;
unsigned long timeMicroseconds = 0UL; // Set in main loop.  Used by several routines.
unsigned long timeMilliseconds = 0UL; // Set in main loop from above.  Used by several routines.

int tpState = 0; // contains system state bits
int cmdState = 0;  // READY, PWR, & HOME command bits

unsigned int forceRight = 0; // force sensor value
unsigned int forceLeft = 0; // force sensor value
double sonarRight = 0.0;
double sonarRightMin = 0.0;
unsigned int sonarFront = 0;
unsigned int sonarLeft = 0;

unsigned int actualLoopTime; // Time since the last
double hcX = 0.0;
double hcY = 0.0;
double pcX = 0.0;
double pcY = 0.0;
double controllerX = 0.0; // +1.0 to -1.0 from controller
double controllerY = 0.0;  // Y value set by message from controller
boolean isNewMessage = false;
char message[100] = "";

double gyroPitchRaw;  // Vertical plane parallel to wheels
double gyroPitchRate;
long gyroPitchRawSum;
double oldGaPitch = 0.0;
double gyroPitchDelta = 0.0;
double gyroPitch = 0.0; // not needed

double gyroRollRaw = 0;
double gyroRollRate;
double gyroRoll = 0.0f;
double accelRoll = 0.0f;

double gyroYawRaw = 0.0f;
double gyroYawRate = 0.0f;
double gyroYawAngle = 0.0f;
double gyroYawRawSum = 0.0;
int gyroErrorX = 0;
int gyroErrorY = 0;
int gyroErrorZ = 0;

int battVolt = 0; // battery 
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
boolean isBluePassthrough = false;

boolean isHcCommand = false;

/* roll pitch and yaw angles computed by iecompass */
int iPhi, iThe, iPsi; 
/* magnetic field readings corrected for hard iron effects and PCB orientation */
int iBfx, iBfy, iBfz;
/* hard iron estimate */
int iVx, iVy, iVz;

double magCorrection = 0.0;

// Sequence variables
int sequenceCount = 0;
boolean sequenceIsRunning = false;
int runSequenceCount = 0;
boolean isSequence = false;
int seqDur = 0;
double seqWs = 0.0;
int seqPw = 0;
double wsArray[30];
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

int wsMFpsRight = 0;
int wsMFpsLeft = 0;
int mFpsRight = 0;
int mFpsLeft = 0;
long wsMFpsRightSum = 0;
long wsMFpsLeftSum = 0;
int wsMFpsRightCount = 0;;
int wsMFpsLeftCount = 0;;
double airFps = 0.0;
unsigned long airTrigger = 0L;

int ackFrameNumber = 0;
int ackFailure = 0;
int ccaFailure = 0;;
int purgeFailure = 0;

double tp5LpfCosAccel = 0.0;
double tp6LpfCosAccel = 0.0;

double tp5FpsLeft = 0.0f;
double tp5FpsRight = 0.0f;
double tp6FpsLeft = 0.0f;
double tp6FpsRight = 0.0f;

int interruptErrorsRight = 0;
int interruptErrorsLeft = 0;

double stopADiff = 0;
double stopDist = 0;

unsigned int xBeeCount = 0;
int rightK = 0;
int leftK = 0;
unsigned int tr;
unsigned int stopTimeRight = UNSIGNED_LONG_MAX;
unsigned int stopTimeLeft = UNSIGNED_LONG_MAX;

int motorRightAction;
int headingSource = HEADING_SOURCE_GM;
boolean isRouteWait = false;
long standPos = 0;
int msgCmdX = 0;
boolean isDiagnostics = false;

boolean isStepFall = false;
boolean isOldStepFall = false;
double tp6ControllerSpeed = 0;
double jumpTarget;
double jumpFallXY;

/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {
  XBEE_SER.begin(57600);  // XBee, See bottom of this page for settings.
  BLUE_SER.begin(115200);  // Bluetooth 
  Serial.begin(115200); // for debugging output
   
  pinMode(LED_PIN,OUTPUT);  // Status LED, also blue LED
  pinMode(LED_PIN,OUTPUT);  // Status LED, also blue LED
  pinMode(YELLOW_LED_PIN,OUTPUT);
  pinMode(RED_LED_PIN,OUTPUT);
  pinMode(GREEN_LED_PIN,OUTPUT);
  pinMode(RIGHT_HL_PIN, OUTPUT);
  pinMode(LEFT_HL_PIN, OUTPUT);
  pinMode(REAR_TL_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  
  pinMode(BATT_PIN, INPUT);
  
  pinMode(SONAR_RIGHT_AN, INPUT);
  pinMode(SONAR_FRONT_AN, INPUT);
  pinMode(SONAR_LEFT_AN, INPUT);
  pinMode(SONAR_RIGHT_RX, OUTPUT);
  pinMode(SONAR_FRONT_RX, OUTPUT);
  pinMode(SONAR_LEFT_RX, OUTPUT);
  digitalWrite(SONAR_RIGHT_RX,HIGH);
  digitalWrite(SONAR_FRONT_RX,HIGH);
  digitalWrite(SONAR_LEFT_RX,HIGH);
  
  pinMode(BU_SW_PIN, INPUT_PULLUP);
  pinMode(YE_SW_PIN, INPUT_PULLUP);
  pinMode(RE_SW_PIN, INPUT_PULLUP);
  pinMode(GN_SW_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(RIGHT_HL_PIN, LOW);
  digitalWrite(LEFT_HL_PIN, LOW);
  digitalWrite(REAR_TL_PIN, LOW);
  digitalWrite(SPEAKER_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);

  Serial.println("Serial & pins initialized.");
  angleInit6();
  Serial.println("Navigation initialized");
  motorInitTp();
  Serial.println("Motors initialized.");
  gyroTrigger = micros();
  beep(BEEP_UP);
  delay(100);
  for (int i = 0; i < DATA_ARRAY_SIZE; i++) {
    aArray[i] = 42;
    bArray[i] = 9;
    cArray[i] = 4242;
    dArray[i] = i;
  }
  zeroGyro();
  Serial.println("Gyro zeroed out.");
  diagnostics();
  Serial.println("Diagnosits ignored.");
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
//  case MODE_TP_SPEED:
//    aTpSpeed();
//    break;
//  case MODE_TP_SEQUENCE:
//    aTpSequence();
//    break;
  case MODE_TP6:
    aTp6Run();
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
  isRunning = true;
  motorInitTp();
//  angleInitTp7();
  setBlink(RED_LED_PIN, BLINK_SB);
  
  while (mode == MODE_PWM_SPEED) {
    commonTasks();
    if (timeMicroseconds > pwmTrigger) {
      int action = FWD;
      pwmTrigger = timeMicroseconds + 50000; // 20/sec
      
      motorMode = vVal;
      if (tVal > 0) action = FWD;
      else action = BKWD;
      setMotor(MOTOR_RIGHT, action, abs(tVal));
      if (uVal > 0) action = FWD;
      else action = BKWD;
      setMotor(MOTOR_LEFT, action, abs(uVal));
      readSpeed();  
          
      sendStatusXBeeHc(); 
      sendStatusBluePc();
    } // end timed loop 
  } // while
} // aPwmSpeed()



/**************************************************************************.
 * aTpSpeed()
 **************************************************************************/
void aTpSpeed() {
  unsigned int loopCount = 0;
  unsigned long tpTrigger = 0;
  tVal = 0;
  uVal = 0;
  isRunning = true;
  motorInitTp();
  setBlink(RED_LED_PIN, BLINK_SF);
  
  while (mode == MODE_TP_SPEED) {
    commonTasks();
    if (timeMicroseconds > tpTrigger) {
      int action = FWD;
      tpTrigger = timeMicroseconds + 2500; // 400/sec
      targetMFpsRight = tVal; // 
      targetMFpsLeft = uVal; // 
      motorMode = vVal;
      readSpeedRight();      
      readSpeedLeft();      
//      checkMotorRight();
//      checkMotorLeft();
      loopCount = ++loopCount % 40; // 1/sec
      if (loopCount == 0) {
        mWheelSpeedFps = mFpsRight;
//        mWheelSpeedFps = mFpsLeft;
//        sendStatusFrame(XBEE_PC);  // Send status message to controller.
//        Serial.print("tVal: ");
//        Serial.print(tVal);
//        Serial.print("\ttargetMFpsRight: "); 
//        Serial.println(targetMFpsRight);
      }
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


/**************************************************************************.
 * zeroGyro()
 **************************************************************************/
void zeroGyro() {
  int loopCount = 400;
  double sumPitch = 0;
  double sumRoll = 0;
  double sumYaw = 0;
  double gyroPitchMin, gyroPitchMax;
  double gyroRollMin, gyroRollMax;
  double gyroYawMin, gyroYawMax;
  unsigned long endTime = millis() + 2000;
  readGyro();
  gyroPitchMin = gyroPitchMax = gyroPitchRaw;
  gyroRollMin = gyroRollMax = gyroRollRaw;
  gyroYawMin = gyroYawMax = gyroYawRaw;
  while(true) {
    if (readGyro()) {
      sumPitch += gyroPitchRaw;
      if (gyroPitchRaw > gyroPitchMax) gyroPitchMax = gyroPitchRaw;
      if (gyroPitchRaw < gyroPitchMin) gyroPitchMin = gyroPitchRaw;
      sumRoll += gyroRollRaw;
      if (gyroRollRaw > gyroRollMax) gyroRollMax = gyroRollRaw;
      if (gyroRollRaw < gyroRollMin) gyroRollMin = gyroRollRaw;
      sumYaw += gyroYawRaw;
      if (gyroYawRaw > gyroYawMax) gyroYawMax = gyroYawRaw;
      if (gyroYawRaw < gyroYawMin) gyroYawMin = gyroYawRaw;
      if (--loopCount <= 0) {
        pitchDrift = (int) (sumPitch / 400.0);
        rollDrift = (int) (sumRoll / 400.0);
        yawDrift = (int) (sumYaw / 400.0);
        break;
      }
      if (millis() > endTime) break;
    }
  }
  Serial.println();
  Serial.print("pitchDrift: "); Serial.print(pitchDrift); Serial.print("\t");
  Serial.print("rollDrift: "); Serial.print(rollDrift); Serial.print("\t");
  Serial.print("yawDrift: "); Serial.print(yawDrift); Serial.println();
  
  Serial.print("Pitch min: "); Serial.print(gyroPitchMin,0); Serial.print("\t");
  Serial.print("Pitch Max: "); Serial.print(gyroPitchMax,0); Serial.print("\t");
  Serial.print("Pitch Range: "); Serial.print(gyroPitchMax - gyroPitchMin,0); Serial.println();
  
  Serial.print("Roll min: "); Serial.print(gyroRollMin,0); Serial.print("\t");
  Serial.print("Roll Max: "); Serial.print(gyroRollMax,0); Serial.print("\t");
  Serial.print("Roll Range: "); Serial.print(gyroRollMax - gyroPitchMin,0); Serial.println();
  
  Serial.print("Yaw min: "); Serial.print(gyroYawMin,0); Serial.print("\t");
  Serial.print("Yaw Max: "); Serial.print(gyroYawMax,0); Serial.print("\t");
  Serial.print("Yaw Range: "); Serial.print(gyroYawMax - gyroYawMin,0); Serial.println();
  
  gPitch = 0.0;
  gRoll = 0.0;
  gYaw = 0.0;
}



/***********************************************************************.
 *  diagnostics()  If the yellow switch is pressed at startup, 
 *                 run diagnosics
 *                 every second which give a printout of the status of
 *                 all systems.
 ***********************************************************************/
void diagnostics() {
  unsigned long dTime1, dTime2;
  Serial.println(digitalRead(BU_SW_PIN));
  if (digitalRead(BU_SW_PIN) != LOW) return;
  isDiagnostics = true;
  Serial.println("Start diagnositics.");

  // Run the motors.
  setMotor(MOTOR_RIGHT, FWD, 70);
  setMotor(MOTOR_LEFT, BKWD, 70);
  
  while (true) {
    delay(1000);

    // Accelerometer
    dTime1 = micros();
    readAccel(); 
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("aPitch: "); Serial.print(aPitch); Serial.print("\t");
    Serial.print("aRoll: "); Serial.print(aRoll); Serial.println();

    // Gyro
    dTime1 = micros();
    readGyro();
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("gyroPitchRate: "); Serial.print(gyroPitchRate); Serial.print("\t");
    Serial.print("gyroRollRate: "); Serial.print(gyroRollRate); Serial.print("\t");
    Serial.print("gyroYawRate: "); Serial.print(gyroYawRate); Serial.println();

    // Magnetometer
    dTime1 = micros();
    readCompass();
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("magHeading: "); Serial.print(magHeading); Serial.println();

    // Ground sensors
    Serial.print("forceRight: "); Serial.print(forceRight); Serial.print("\t");
    Serial.print("forceLeft: "); Serial.print(forceLeft); Serial.println();

    // Sonar
    double rSonar = analogRead(SONAR_RIGHT_AN) * SONAR_SENS;
    Serial.print("Sonar: "); Serial.print(rSonar); Serial.println();

    // Motors
    readSpeedRight();
    readSpeedLeft();
    Serial.print("fpsRight: "); Serial.print(fpsRight); Serial.print("\t");
    Serial.print("fpsLeft: "); Serial.print(fpsLeft); Serial.println();

    
    // XBee & Bluetooth
//    dTime1 = micros();
//    unsigned long cEnd = millis() + 100;
//    sendStatusBluePc();
//    while(true) {
//      timeMilliseconds = millis();
//      readBluetooth();
//      if (
//      if (millis() > cEnd) break;
//    }
//    sendStatusXBeeHc() {
//
    Serial.println();
  }
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











