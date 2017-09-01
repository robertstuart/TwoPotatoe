/* ---------------------- TwoPotatoe ----------------------- */

#include "Common.h"
#include <DueTimer.h>
#include <Wire.h>
#include <LSM6.h>

// One screw turn on the stand produces 4" change at 100'

// Decrease this value to get greater turn for a given angle
//#define GYRO_SENS 0.0690
//#define GYRO_SENS 0.0696     
const float GYRO_SENS = 0.0696;      // Multiplier to get degrees. 

#define TICKS_PER_FOOT 2222.0D // For Losi DB XL 1/5 scale

// Values for initial timeDrift???
const double PITCH_DRIFT = -26.63;
const double ROLL_DRIFT = 63.60;
const double YAW_DRIFT = -23.47;

#define XBEE_SER Serial3
#define BLUE_SER Serial1
#define SONAR_SER Serial2

//#define TICKS_PER_CIRCLE_YAW  11900.0  // For Pro-Line Masher 2.8" PRO1192-12
//#define TICKS_PER_CIRCLE_YAW  7816.0  // For Losi DB XL 1/5 scale
//#define TICKS_PER_CIRCLE_YAW  7428.0  // For Losi DB XL 1/5 scale

LSM6 lsm6;

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_RIGHT_ENCA =  22;  
const int MOT_LEFT_ENCA =   23;   
const int MOT_RIGHT_ENCB =  24; 
const int MOT_LEFT_ENCB =   25; 
const int MOT_RIGHT_ENCZ =  26; 
const int MOT_LEFT_ENCZ =   27; 

const int MOT_RIGHT_DIR =    4;   
const int MOT_LEFT_DIR =     5 ; 
const int MOT_RIGHT_PWMH =   6; 
const int MOT_LEFT_PWMH =    7;  

const double SPEED_MULTIPLIER = 20.0;
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

//#define TICKS_PER_PITCH_DEGREE 54.0D
//#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer
#define GYRO_WEIGHT 0.997    // Weight for gyro compared to accelerometer
#define DEFAULT_GRID_OFFSET 0.0
#define SONAR_SENS 0.0385

#define INVALID_VAL -123456.78D

// Due has 96 kbytes sram
//#define DATA_ARRAY_SIZE 4000
#define DATA_ARRAY_SIZE 2500
// Arrays to save data to be dumped in blocks.
long  aArray[ DATA_ARRAY_SIZE];
short bArray[ DATA_ARRAY_SIZE];
short cArray[ DATA_ARRAY_SIZE];
short dArray[ DATA_ARRAY_SIZE];
short eArray[ DATA_ARRAY_SIZE];
short fArray[ DATA_ARRAY_SIZE];
short gArray[ DATA_ARRAY_SIZE];
unsigned int dataArrayPtr = 0;

#define TICK_ARRAY_SIZE 1030
short tArray[TICK_ARRAY_SIZE]; // Period in usec
short uArray[TICK_ARRAY_SIZE]; // State of rotation tick
unsigned int tickArrayPtr = 0;

//int sumYaw = 0;

unsigned int mode = MODE_2P;
boolean motorMode = MM_DRIVE_BRAKE;  // Can also be MM_DRIVE_COAST
//unsigned int mode = MODE_TP_SPEED;
unsigned int oldMode = MODE_TP5;  // Used by PULSE_SEQUENCE

volatile int beepEnd = 0;
volatile int beepPeriod = 0;
volatile int beepPeriodCount = 0;
int BEEP_UP [] = {1200, 100, 1500, 100, 0}; // pitch, time, pitch, time, etc.
int BEEP_UP2 [] = {1500, 100, 1800, 100, 0}; // pitch, time, pitch, time, etc.
//int BEEP_UP [] = {600, 400, 800, 400, 1000, 400, 1200, 400, 1400, 400, 1600, 400, 0}; 
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
  float t;
  float u;
  float v;
  float w;
  float x;
  float y;
  float z;
};

struct valSet tp7 = { 
  4.5,     // t
  0.98,    // u
  2.7,     // v
  0.95,    // w
  2.0,     // x
  0.2,     // y
  0.0
}; 

valSet *currentValSet = &tp7;
int vSetStatus = VAL_SET_A;
int bbb = 42;
 

struct loc {
  float x;
  float y;
};

struct loc currentLoc;
struct loc targetLoc;
struct loc pivotLoc;
struct loc hugStartLoc;
//boolean isFixLoc = false;
struct loc coSetLoc;
boolean isFixHeading = false;
boolean isAngleControl = false;
boolean decelStopped = false;

//double lsXYSurface = 0;
//double hugXYRhumb = 0.0D;
//double hugXYSurface = 0.0D;
//double hugSonarDistance = 0.0D;
//double hugBearing = 0.0D;
//char hugDirection = 'N';
//boolean isHug = false;
boolean isLockStand = true;

int routeStepPtr = 0;
String routeTitle = "no title                ";
int dum1 = 2;
int dum2 = 3;
boolean isDecelPhase = false;    // Reached point where dece starts?
boolean isDecelActive = false;   // Decelerate for this G or T?
float decelFps = 0.0;
float tpFps = 0.0;

double turnTargetCumHeading = 0.0;

char routeCurrentAction = 0;
boolean isRightTurn = true;
float routeFps = 0.0;
float routeScriptFps = 0.0;  // The programmed speed from the script.
float turnRadius = 0.0;
float pivotBearing = 0.0;
float endTangentDegrees = 0.0;
int routeWaitTime = 0L;
boolean isStartReceived = false;
//double routeTargetXYDistance = 0.0;
int originalAction = 0;

double gaPitch = 0.0;
float yAccel = 0.0;
//double gaFullPitch = 0.0;
double lpfAPitch = 0.0;
double gaRoll = 0.0;

double aPitch = 0.0;
double aRoll = 0.0;

int baseFahr = 0;
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
boolean isLifted = true;
boolean isOnGround = false;
boolean isHcActive = false; // Hand controller connected.
boolean isPcActive = false; // PC connected
boolean isRouteInProgress  = false; // Route in progress
boolean isDumpingData = false; // Dumping data
boolean isHoldHeading = false; // 
boolean isSpin = false; // 
boolean isStand = false; // 

boolean isBackLeft = false;


volatile boolean isNewCheck = false;

boolean isDumpingTicks = false;

int standTPRight = 0;
int standTPLeft = 0;

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

unsigned int forceRight = 0; // force sensor value
unsigned int forceLeft = 0; // force sensor value
float sonarLeft = 0.0;
float sonarLeftKeep = 0.0;
float sonarFront = 0.0;
float sonarFrontKeep = 0.0;
float sonarRight = 0.0;
float sonarRightKeep = 0.0;

unsigned int actualLoopTime; // Time since the last
double hcX = 0.0;
double hcY = 0.0;
double pcX = 0.0;
double pcY = 0.0;
double controllerX = 0.0; // +1.0 to -1.0 from controller
double controllerY = 0.0;  // Y value set by message from controller
char message[100] = "";
char message1[80] = "message 1";
char message2[80] = "message 2";
char message3[80] = "message 3";
char message4[80] = "message 4";
float baseGyroTemp = 75.0;
double gyroPitchRaw;  // Vertical plane parallel to wheels
double gyroPitchRate;
double oldGaPitch = 0.0;
double gyroPitchDelta = 0.0;
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

double stopADiff = 0;
double stopDist = 0;

unsigned int xBeeCount = 0;
int rightK = 0;
int leftK = 0;
unsigned int tr;
unsigned int stopTimeRight = UNSIGNED_LONG_MAX;
unsigned int stopTimeLeft = UNSIGNED_LONG_MAX;

int motorRightAction;
boolean isRouteWait = false;
long standPos = 0;
int msgCmdX = 0;
boolean isDiagnostics = false;

double tp6ControllerSpeed = 0.0;
float tp7ControllerSpeed = 0.0;
double jumpTarget;
double jumpFallXY;

float barrelXCenter = 0.0D;
float barrelX = 0.0D;
float barrelYEnd = 0.0D;

char pBuf[100];

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
  SONAR_SER.begin(9600);   // Mini-pro sonar controller
  Serial.begin(115200); // for debugging output
   
  pinMode(LED_PIN,OUTPUT);  // Status LED, also blue LED
  pinMode(RED_LED_PIN,OUTPUT);
  pinMode(YELLOW_LED_PIN,OUTPUT);
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
  Timer3.attachInterrupt(pollIsr);
  Timer3.start(100);  // Poll at 10,000/sec.
//  diagnostics();
  Serial.println("Diagnostics ignored.");
  setSonar("LFR");
  sonarSlope();
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
  case MODE_2P:
    aTp7Run();
    break;
  default:
    readXBee();
    break;
  } // end switch(mode)
} // End loop().  



/************************************************************************** *
 * pollIsr()  Called every 100 microSeconds.
 *            Call the beep and checkMotor() routines.
 *********************************************************/
void pollIsr() {
  beepIsr();
}


/**************************************************************************.
 * aPwmSpeed()
 **************************************************************************/
void aPwmSpeed() {
  int loop = 0;
  float sumRight, sumLeft;
  setBlink(RED_LED_PIN, BLINK_SB);
  beep(BEEP_UP2);
  tVal = uVal = 0;
  
  while (mode == MODE_PWM_SPEED) {
    commonTasks();
    if (isNewCheck) {  // 1000/sec
      isNewCheck = false;
      setMotorRight(abs(tVal), (tVal >= 0) ? FWD : BKWD);
      setMotorLeft(abs(uVal), (uVal >= 0) ? FWD : BKWD);
      sumRight += wFpsRight;
      sumLeft += wFpsLeft;
      if (!(loop++ % 1000)) {  
        float r = sumRight / 1000.0;
        float l = sumLeft / 1000.0;
        sumRight = sumLeft = 0.0;
        sprintf(message, "%5.2f   %5.2f  %5.2f", l, r, wFps);
        sendBMsg(SEND_MESSAGE, message);
      }
    } // end timed loop 
  } // while
} // aPwmSpeed()



/**************************************************************************.
 * aTpSpeed()
 **************************************************************************/
void aTpSpeed() {
  unsigned int loop = 0;
  float sumRight = 0.0, sumLeft = 0.0;
  setBlink(RED_LED_PIN, BLINK_SF);
  beep(BEEP_UP2);
  tVal = uVal = 0;
  
  while (mode == MODE_TP_SPEED) {
    commonTasks();
    if (isNewCheck) {
      isNewCheck = false;
      targetWFpsRight = ((float) tVal) / 1000.0; // 
      targetWFpsLeft = ((float) uVal) / 1000.0; // 
      sumRight += wFpsRight;
      sumLeft += wFpsLeft;
      if (!(loop++ % 1000)) {  
        float r = sumRight / 1000.0;
        float l = sumLeft / 1000.0;
        sumRight = sumLeft = 0.0;
        sprintf(message, "%5.2f   %5.2f", l, r);
        sendBMsg(SEND_MESSAGE, message);
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
//  pulseIndex = 0;
//  pulseTrigger = 0L;
//  
//  while (mode == MODE_PULSE_SEQUENCE) {
//    timeMicroseconds = micros();
//    if (timeMicroseconds > pulseTrigger) {
//      if (pulseIndex == pulseCount) { // End of sequence      
//        mode = oldMode;
//        if (isPwData) {
//          isPwData = false;
//          sendDumpData();
//        }
//      }
//      else {
//        int motor = motorArray[pulseIndex];
//        int pw = pwArray[pulseIndex];
//        pulseTrigger = timeMicroseconds + pw;
//        pulseIndex++;
//        switch (motor) {
//        case 1: // Coast
//          setMotor(MOTOR_RIGHT, COAST, 0);
//          break;
//        case 2: // Forward
//          setMotor(MOTOR_RIGHT, FWD, 255);
//          break;
//        case 3: // Backwards
//          setMotor(MOTOR_RIGHT, BKWD, 255);
//          break;
//        default: // Brake
//          setMotor(MOTOR_RIGHT, BRAKE, 0);
//          break;
//        }
//      }
//    } 
//  }
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
  setMotorRight(1000, FWD);
  setMotorLeft(1000, FWD);
  
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
    Serial.print("wFpsRight: "); Serial.print(wFpsRight); Serial.print("\t");
    Serial.print("wFpsLeft: "); Serial.print(wFpsLeft); Serial.println();

    
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











