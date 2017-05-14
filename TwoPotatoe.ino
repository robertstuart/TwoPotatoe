/* ---------------------- TwoPotatoe ----------------------- */

#include "Common.h"
#include "pwm01.h"
#include <DueTimer.h>
#include <Wire.h>
#include <LSM6.h>

// One screw turn on the stand produces 4" change at 100'

// Decrease this value to get greater turn for a given angle
//#define GYRO_SENS 0.0690     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
#define GYRO_SENS 0.0696     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec

//#define TICKS_PER_FOOT 3017.0D  // For G-made inflatable
//#define TICKS_PER_FOOT 3342.0D // For Pro-Line Masher 2.8" PRO1192-12
#define TICKS_PER_FOOT 2222.0D // For Losi DB XL 1/5 scale

// Values for initial timeDrift???
const double PITCH_DRIFT = -16.52;
const double ROLL_DRIFT = 63.60;
const double YAW_DRIFT = -26.81;

#define XBEE_SER Serial3
#define BLUE_SER Serial1
#define SONAR_SER Serial2

//#define TICKS_PER_CIRCLE_YAW  11900.0  // For Pro-Line Masher 2.8" PRO1192-12
//#define TICKS_PER_CIRCLE_YAW  7816.0  // For Losi DB XL 1/5 scale
#define TICKS_PER_CIRCLE_YAW  7428.0  // For Losi DB XL 1/5 scale

LSM6 lsm6;

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
//const unsigned int TP_PWM_FREQUENCY = 1000;

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

#define SONAR_NONE 0
#define SONAR_RIGHT 1
#define SONAR_LEFT 2
#define SONAR_BOTH 3

#define LED_PIN         13 // LED connected to digital pin 13
#define BLUE_LED_PIN    13 // LED in switch, same as status above
#define YELLOW_LED_PIN  12 // LED in switch
#define RED_LED_PIN     11 // LED in switch
#define RIGHT_HL_PIN    10 // headlamp
#define LEFT_HL_PIN      9 // headlamp
#define REAR_TL_PIN      8 // rear lamp
#define GREEN_LED_PIN   38 // LED in switch
#define ACCEL_INTR_PIN  31
#define GYRO_INTR_PIN   30

#define BU_SW_PIN 39 // Blue switch
#define YE_SW_PIN 34 // Yellow switch
#define RE_SW_PIN 37 // Red switch
#define GN_SW_PIN 35 // Green switch

#define SPEAKER_PIN 36 // 
#define BATT_PIN A0
#define R_FORCE_PIN A1             // Force sensor
#define L_FORCE_PIN A2             // Force sensor
//#define R_CURRENT_PIN A3             // Right motor current
//#define L_CURRENT_PIN A4             // Left motor current

#define SONAR_RIGHT_PIN 45
#define SONAR_LEFT_PIN 43

#define A_LIM 20.0 // degrees at which the speedAdjustment starts reducing.
#define S_LIM 1.0  // maximum speedAdjustment;

String tab = "\t";

//Encoder factor
//const double ENC_FACTOR = 1329.0f;  // Change pulse width to fps speed, 1/29 gear
//const long ENC_FACTOR_M = 1329000L;  // Change pulse width to milli-fps speed, 1/29 gear
const double ENC_FACTOR = 650.0f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 650000L;  // Change pulse width to milli-fps speed
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
#define DEFAULT_GRID_OFFSET 0.0
#define SONAR_SENS 0.0385

#define INVALID_VAL -123456.78D

// Due has 96 kbytes sram
#define DATA_ARRAY_SIZE 2500
//#define DATA_ARRAY_SIZE 20
// Arrays to save data to be dumped in blocks.
long  aArray[ DATA_ARRAY_SIZE];
short bArray[ DATA_ARRAY_SIZE];
short cArray[ DATA_ARRAY_SIZE];
short dArray[ DATA_ARRAY_SIZE];
short eArray[ DATA_ARRAY_SIZE];
short fArray[ DATA_ARRAY_SIZE];
short gArray[ DATA_ARRAY_SIZE];
unsigned int dataArrayPtr = 0;

//#define TICK_ARRAY_SIZE 10300
#define TICK_ARRAY_SIZE 1030
short tArray[TICK_ARRAY_SIZE]; // Period in usec
short uArray[TICK_ARRAY_SIZE]; // State of rotation tick
unsigned int tickArrayPtr = 0;

//int sumYaw = 0;

unsigned int mode = MODE_TP6;
boolean motorMode = MM_DRIVE_BRAKE;  // Can also be MM_DRIVE_COAST
//unsigned int mode = MODE_TP_SPEED;
unsigned int oldMode = MODE_TP5;  // Used by PULSE_SEQUENCE

int BEEP_UP [] = {1200, 100, 1500, 100, 0};
int BEEP_WARBLE[] = {2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 0};
int BEEP_DOWN[] = {1000, 200, 666, 200, 0};

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

struct valSet tp6 = { 
  6.0,    // t
  0.1,    // u
  4.0,    // v - was 2.0, set to 3.0 for low speed
  0.18,    // w
  0.05,    // x
 130.0,   // y
 -1.6}; // z accelerometer offset

valSet *currentValSet = &tp6;
int vSetStatus = VAL_SET_A;
int bbb = 42;
 

struct loc {
  double x;
  double y;
};

struct loc currentMapLoc;
struct loc routeTargetLoc;
//boolean isFixLoc = false;
struct loc coSetLoc;
boolean isFixHeading = false;
boolean isAngleControl = false;
boolean decelStopped = false;

struct chartedObject {
  boolean isRightSonar;
  char type;            // 'S', 'H', or ' ',
  double trigger;
  double surface;
};

#define SONAR_ARRAY_SIZE 100
int sonarRightArray[SONAR_ARRAY_SIZE];
int sonarLeftArray[SONAR_ARRAY_SIZE];
int sonarRightArrayPtr = 0;
int sonarLeftArrayPtr = 0;

#define CO_SIZE 10
struct chartedObject chartedObjects[CO_SIZE];
int coPtr = 0;
int coEnd = 0;

#define LS_ARRAY_SIZE 1000
unsigned int lsPtr = 0;
float lsXArray[LS_ARRAY_SIZE] = {5,7,9,12,15};
float lsYLArray[LS_ARRAY_SIZE] = {2,2.2,2.5,2.6,3};
float lsSArray[LS_ARRAY_SIZE] = {1.9,2.1,2.6,2.7,3.2};
double lsXYSurface = 0;
double hugXYRhumb = 0.0D;
double hugXYSurface = 0.0D;
double hugSonarDistance = 0.0D;
double hugBearing = 0.0D;
char hugDirection = 'N';
boolean isHug = false;
boolean isLockStand = true;
int lockStartTicks = 0;

int routeStepPtr = 0;
String routeTitle = "No route";

boolean isTurnDegrees = false;
double turnTargetCumHeading = 0.0;
struct loc pivotLoc;

char routeCurrentAction = 0;
double routeTargetBearing = 0.0;
//double phantomTargetBearing = 0.0;
double routeMagTargetBearing = 0.0;
boolean isReachedMagHeading = false;
boolean routeIsRightTurn = true;
long routeTargetTickPosition = 0L;
double targetDistance = 0.0D;
double pirouetteFps = 0.0;
double routeFps = 0.0;
double stickSpeed = 0.0;
double routeRadius = 0.0;
int routeWaitTime = 0L;
boolean isEsReceived = false;
boolean isRouteTargetIncreasing = false;
boolean isGXAxis = false;
double routeTargetXY = 0.0;
long navOldTickPosition = 0L;
double currentMapHeading = 0.0;
double currentMapCumHeading = 0.0;
double routeTargetXYDistance = 0.0;
int originalAction = 0;

double sumX = 0.0D;
double sumY = 0.0D;
double sumZ = 0.0D;
double meanX = 0.0D;
double meanY = 0.0D;
double meanZ = 0.0D;

double gaPitch = 0.0;
double gaFullPitch = 0.0;
double tgaPitch = 0.0;
double gaRoll = 0.0;

double aPitch = 0.0;
double aRoll = 0.0;

int baseFahr = 0;
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

double xVec, yVec, zVec;
boolean isMagAdjust = true;
double magHeading = 0.0;
double gridOffset = 0.0;
double gridHeading = 0.0;
double gridCumHeading = 0.0;
double gridRotations = 0.0;
double tickHeading = 0.0;
double tickCumHeading = 0.0;
int tickOffset = 0;
double tmHeading = 0.0;
double tmCumHeading = 0.0;
double gmHeading = 0.0;
double gmCumHeading = 0.0;
double currentX = 0.0;
double currentY = 0.0;
int fixPosition = 0;
double fixHeading = 0.0;

// Speed and position variables
int tickPositionRight = 0;
int tickPositionLeft = 0;
long tickPosition;
int lastTickSet = 0;
  

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
boolean isStand = false; // 

boolean isDumpingTicks = false;

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

unsigned long statusTrigger = 0UL;
unsigned long oldTimeTrigger = 0UL;
unsigned long timeMicroseconds = 0UL; // Set in main loop.  Used by several routines.
unsigned long timeMilliseconds = 0UL; // Set in main loop from above.  Used by several routines.

int tpState = 0; // contains system state bits
int cmdState = 0;  // READY, PWR, & HOME command bits

unsigned int forceRight = 0; // force sensor value
unsigned int forceLeft = 0; // force sensor value
float sonarLeft = 0.0;
float sonarLeftMin = 0.0;
float sonarFront = 0.0;
float sonarLeftFront = 0.0;
float sonarRight = 0.0;
float sonarRightMin = 0.0;
double sonarMin = 0.0D;
double sonarMax = 100.0D;

unsigned int actualLoopTime; // Time since the last
double hcX = 0.0;
double hcY = 0.0;
double pcX = 0.0;
double pcY = 0.0;
double controllerX = 0.0; // +1.0 to -1.0 from controller
double controllerY = 0.0;  // Y value set by message from controller
//boolean isNewMessage = false;
char message[100] = "";

float baseGyroTemp = 75.0;
double gyroPitchRaw;  // Vertical plane parallel to wheels
double gyroPitchRate;
double oldGaPitch = 0.0;
double gyroPitchDelta = 0.0;
double gyroPitch = 0.0; // not needed
double timeDriftPitch = PITCH_DRIFT;

double gyroRollRaw = 0;
double gyroRollRate;
double gyroRoll = 0.0f;
double accelRoll = 0.0f;
double timeDriftRoll = ROLL_DRIFT;

double gyroYawRaw = 0.0D;
double gyroYawRate = 0.0f;
double gyroYawAngle = 0.0f;
double gyroYawRawSum = 0.0;
double temperatureDriftYaw = 0.0D;
double timeDriftYaw = YAW_DRIFT;

int gyroErrorX = 0;
int gyroErrorY = 0;
int gyroErrorZ = 0;

float battVolt = 0; // battery 
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
//boolean isBluePassthrough = false;

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
int headingSource = HEADING_SOURCE_G;
boolean isRouteWait = false;
long standPos = 0;
int msgCmdX = 0;
boolean isDiagnostics = false;

boolean isStepFall = false;
boolean isOldStepFall = false;
double tp6ControllerSpeed = 0;
double jumpTarget;
double jumpFallXY;

double routeTargetMagHeading = 0.0;
char pBuf[100];
/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {
  resetIMU();
  
  XBEE_SER.begin(57600);  // XBee, See bottom of this page for settings.
  BLUE_SER.begin(115200);  // Bluetooth 
  SONAR_SER.begin(9600);   // Mini-pro sonar controller
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

  pinMode(R_FORCE_PIN, INPUT);
  pinMode(L_FORCE_PIN, INPUT);

  pinMode(SONAR_RIGHT_PIN, OUTPUT);
  pinMode(SONAR_LEFT_PIN, OUTPUT);
  
  pinMode(BU_SW_PIN, INPUT_PULLUP);
  pinMode(YE_SW_PIN, INPUT_PULLUP);
  pinMode(RE_SW_PIN, INPUT_PULLUP);
  pinMode(GN_SW_PIN, INPUT_PULLUP);
  pinMode(GYRO_INTR_PIN, INPUT);
  pinMode(ACCEL_INTR_PIN, INPUT);

  pinMode(MOT_RIGHT_ENCA, INPUT);
  pinMode(MOT_RIGHT_ENCB, INPUT);
  pinMode(MOT_RIGHT_ENCZ, INPUT);
  pinMode(MOT_LEFT_ENCA, INPUT);
  pinMode(MOT_LEFT_ENCB, INPUT);
  pinMode(MOT_LEFT_ENCZ, INPUT);

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
  delay(100);
  for (int i = 0; i < DATA_ARRAY_SIZE; i++) {
    aArray[i] = 42;
    bArray[i] = 9;
    cArray[i] = 4242;
    dArray[i] = i;
  }
//  zeroGyro();
//  Serial.println("Gyro zeroed out.");
//  diagnostics();
  Serial.println("Diagnostics ignored.");
  beep(BEEP_UP);
  setSonar("LFR");
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
      if (isDumpingTicks) dumpTicks();
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
    isNewAccel();
    setAccelData(); 
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("aPitch: "); Serial.print(aPitch); Serial.print("\t");
    Serial.print("aRoll: "); Serial.print(aRoll); Serial.println();

    // Gyro
    dTime1 = micros();
    isNewGyro();
    setGyroData();
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("gyroPitchRate: "); Serial.print(gyroPitchRate); Serial.print("\t");
    Serial.print("gyroRollRate: "); Serial.print(gyroRollRate); Serial.print("\t");
    Serial.print("gyroYawRate: "); Serial.print(gyroYawRate); Serial.println();

    // Magnetometer
    dTime1 = micros();
//    readCompass();
//    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
//    Serial.print("magHeading: "); Serial.print(magHeading); Serial.println();

    // Ground sensors
    Serial.print("forceRight: "); Serial.print(forceRight); Serial.print("\t");
    Serial.print("forceLeft: "); Serial.print(forceLeft); Serial.println();

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











