/*****************************************************************************-
 *  *  Angle
 *****************************************************************************/
//#define I2Cclock 400000
//#define I2Cport Wire
#define MPU9250_ADDRESS   MPU9250_ADDRESS_AD0 // 0x68

MPU9250 imu(MPU9250_ADDRESS, Wire, 400000);

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
int zSum = 0;
int zCount = 0;
int yawTempComp = 0;


/*****************************************************************************-
    imuInit()
 ***********************************************************************/
void imuInit() {
  //  Wire.begin();
  pinMode(IMU_INT_PIN, INPUT);

  byte b = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (b == 0x71) {
    Serial.println("MPU9250 on-line.");
  } else {
    Serial.println("MPU9250 off-line.");
//    while (true) {}
  }
  
  imu.writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);  // Reset
  delay(100);
  imu.writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  //
  delay(200);
  imu.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 3);       // 250/sec
  imu.writeByte(MPU9250_ADDRESS, CONFIG, 1);           // BW=184HZ, delay=2.9ms
  imu.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x18);   //2000 dps
  imu.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x10);  // 8g FS
  imu.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03); // BW=41HZ, delay=11.8
  // Int active low, push-pull, hold int until clear, clear on read
  imu.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x32);  //0x22 works, 0x020 bit must be set!
  imu.writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // int on data ready
}


#define TG_PITCH_TC 0.90D



/*****************************************************************************-
    isNewImu()   Returns true if the IMU has new data.
                Reads the IMU and sets the new values.
 *****************************************************************************/
boolean isNewImu() {
  if (digitalRead(IMU_INT_PIN) == HIGH) {
    unsigned int t = micros();
    imu.readGyroData(imu.gyroCount);
    imu.readAccelData(imu.accelCount);

    gyroRollRaw = imu.gyroCount[0];
    gyroPitchRaw = imu.gyroCount[1];
    gyroYawRaw = imu.gyroCount[2];
    gyroPitchRate = (((float) gyroPitchRaw) - timeDriftPitch)* GYRO_SENS; // 
    gyroRollRate = (((float) gyroRollRaw) - timeDriftRoll) * GYRO_SENS;
    gyroYawRate = (((float) gyroYawRaw) - timeDriftYaw) * GYRO_SENS;
    accelX = ((float) imu.accelCount[0]) * ACCEL_SENS;
    accelY = ((float) imu.accelCount[1]) * ACCEL_SENS;
    accelZ = ((float) imu.accelCount[2]) * ACCEL_SENS;
    
    doGyro();
    doAccel();
    return true;
  } else {
    return false;
  }
  
}



/*****************************************************************************-
    doGyro
 *****************************************************************************/
void doGyro() {

  // Pitch
  gyroPitchDelta = -gyroPitchRate / 250.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  //  gaFullPitch = gyroPitchDelta + gaFullPitch;

  // Roll
  float gyroRollDelta = gyroRollRate / 255.0;
  gRoll = gRoll + gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
  double gyroYawDelta = -gyroYawRate / 255.0; // degrees changed during period
  gYaw += gyroYawDelta;
  gHeading = rangeAngle(gYaw);  // This is the heading used by all navigation routines.

  // Tilt compensated heading
  double yawDeltaError = gyroYawDelta * gaPitch * gaPitch * 0.000135;
  gcYaw += gyroYawDelta + yawDeltaError;
  gcHeading = rangeAngle(gcYaw);  // This is the heading used by all navigation routines.
}


static double APITCH_TC = 0.99;
/*****************************************************************************-
     doAccel()
 *****************************************************************************/
void doAccel() {
  double oldLpfAPitch = 0;
  // Pitch
  //  double k8 = 2.0;  //
  //  double k8 = valV;
  aPitch = ((atan2(accelX, accelZ)) * RAD_TO_DEG) + valZ;
  gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  lpfAPitch = (oldLpfAPitch * APITCH_TC) + (aPitch * (1.0 - APITCH_TC));
  oldLpfAPitch = lpfAPitch;

  // Roll
  aRoll =  (atan2(accelY, accelZ) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}


//#define GM_HEADING_TC 0.95D
#define GM_HEADING_TC 0.98D
#define TM_HEADING_TC 0.999D

int magX, magY, magZ;



/*****************************************************************************-
 *   setNavigation() Set gmHeading, tmHeading, tickHeading, currentLoc
 *                  Called 208/sec (every read of gyro).
 ***********************************************************************/
#define TICK_BIAS_TRIGGER 500
void setNavigation() {
  static int navOldTickPosition = 0;

  tickPosition = tickPositionRight + tickPositionLeft;

  // compute the Center of Oscillation Tick Position
  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

  // Compute the new co position
  double dist = ((double) (coTickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  navOldTickPosition = coTickPosition;
  currentLoc.x += sin(gHeading * DEG_TO_RAD) * dist;
  currentLoc.y += cos(gHeading * DEG_TO_RAD) * dist;

  currentAccelLoc();
}




//double accelFpsSelfX = 0.0;
//double accelFpsSelfY = 0.0;
//double accelFpsMapX = 0.0;
//double accelFpsMapY = 0.0;
//struct loc currentAccelSelfLoc;
//struct loc currentAccelMapLoc;

const double A_FACTOR = .000001D;
/*****************************************************************************-
 *  setAccelLoc() Set currentAccelLoc
 *****************************************************************************/
void currentAccelLoc() {
  //  currentAccelMapLoc.y += ((double) compass.a.y) * A_FACTOR;
  //  currentAccelMapLoc.x += ((double) compass.a.x) * A_FACTOR;
  //  currentAccelMapLoc.x += accelFpsSelfX * .0025;
  //  currentAccelMapLoc.y += accelFpsSelfY * .0025;
  //  accelFpsMapX = (sin(currentMapHeading * DEG_TO_RAD) * accelFpsSelfX) + (cos(currentMapHeading * DEG_TO_RAD) * accelFpsSelfY);
  //  accelFpsMapY = (cos(currentMapHeading * DEG_TO_RAD) * accelFpsSelfX) + (sin(currentMapHeading * DEG_TO_RAD) * accelFpsSelfY);
  //  currentAccelMapLoc.x += accelFpsMapX * 0.0025;
  //  currentAccelMapLoc.y += accelFpsMapY * 0.0025;
}



/*****************************************************************************-
 * setHeading() Sets the bearing to the new value.  The the gridOffset
 *              value will be set so that the gridBearing is an
 *              offset from magHeading.  All of cumulative rotations
 *              be lost.
 **************************************************************************/
void setHeading(double newHeading) {
  gcHeading = gHeading = gcYaw = gYaw = newHeading;
}

void resetTicks() {
  tickPosition = tickPositionRight = tickPositionLeft = coTickPosition = 0;
}


/**************************************************************************.
   readHMC() Routine taken from CalibrateHMC
 **************************************************************************/
//boolean readHMC() {
//  int xab, yab, zab;
//  unsigned int timeOut = 2;
//
//  //Tell the HMC5883 where to begin reading data
//  Wire.beginTransmission(HC_ADDRESS);
//  Wire.write(0x03); //select register 3, X MSB register
//  Wire.endTransmission();
//  Wire.requestFrom(HC_ADDRESS, 6);
//
//  unsigned int millisStart = millis();
//
//  //Read data from each axis, 2 registers per axis
//  while (Wire.available() < 6) {
//    if ((timeOut > 0) && ((millis() - millisStart) > timeOut)) {
//      return false;
//    }
//  }
//
//  byte xhm = Wire.read(); //X msb
//  byte xlm = Wire.read(); //X lsb
//  byte zhm = Wire.read(); //Z msb
//  byte zlm = Wire.read(); //Z lsb
//  byte yhm = Wire.read(); //Y msb
//  byte ylm = Wire.read(); //Y lsb
//
//  xab = (int16_t)(xhm << 8 | xlm);
//  yab = (int16_t)(yhm << 8 | ylm);
//  zab = (int16_t)(zhm << 8 | zlm);
//
//  if ((xab != magX) || (yab != magY) || (zab != magZ)) {
//    magX = xab;
//    magY = yab;
//    magZ = zab;
//    return true;
//  }
//  return false;
//}



const int DRIFT_SIZE = 125;
//float xDriftSum = 0.0;
//float yDriftSum = 0.0;
//float zDriftSum = 0.0;
//int gPtr = 0;
//boolean isMeasuringDrift = false;
int zArray[DRIFT_SIZE];
int yArray[DRIFT_SIZE];
int xArray[DRIFT_SIZE];
/*****************************************************************************-
 * doGyroDrift()  Called 250/sec.  Average gyroDrift for x, y & z
 *                for 1/2 second periods.
 *****************************************************************************/
void doGyroDrift() {
  static int gPtr = 0;
  int x, y, z;

  xArray[gPtr] = gyroPitchRaw;
  yArray[gPtr] = gyroRollRaw;
  zArray[gPtr] = gyroYawRaw;
  gPtr++;
  if (gPtr >= DRIFT_SIZE) {
    gPtr = 0;
    int xSum = 0;
    int ySum = 0;
    int zSum = 0;
    int xMax = xArray[0];
    int xMin = xMax;
    int yMax = yArray[0];
    int yMin = yMax;
    int zMax = zArray[0];
    int zMin = zMax;
    for (int i = 0; i < DRIFT_SIZE; i++) {
      x = xArray[i];
      if (x > xMax)  xMax = x;
      if (x < xMin)  xMin = x;
      xSum += x;
      y = yArray[i];
      if (y > yMax)  yMax = y;
      if (y < yMin)  yMin = y;
      ySum += y;
      z = zArray[i];
      if (z > zMax)  zMax = z;
      if (z < zMin)  zMin = z;
      zSum += z;
    }
    float xAve = ((float) xSum) / ((float) DRIFT_SIZE);
    float yAve = ((float) ySum) / ((float) DRIFT_SIZE);
    float zAve = ((float) zSum) / ((float) DRIFT_SIZE);

    // If we have a stable 0.5 second period, average the most recent 20 periods & adjust drift.
    if (((xMax - xMin) < 15) && ((yMax - yMin) < 15) && ((zMax - zMin) < 15)) {
      setDrift(xAve, yAve, zAve);
    }
//    sprintf(message, "xMin: %4d     xMax: %4d     xAve: %5.1f   ", xMin, xMax, xAve);
//    Serial.print(message);
//    sprintf(message, "yMin: %4d     yMax: %4d     yAve: %5.1f   ", yMin, yMax, yAve);
//    Serial.print(message);
//    sprintf(message, "zMin: %4d     zMax: %4d     zAve: %5.1f\n", zMin, zMax, zAve);
//    Serial.print(message);
  }
}

#define AVE_SIZE 20  // Equals 10 seconds of measurements
/*****************************************************************************-
 *  setDrift()  Called to add the last 1/2 sec of drift values.
 *****************************************************************************/
void setDrift(float xAve, float yAve, float zAve) {
  static float xAveArray[AVE_SIZE];
  static float yAveArray[AVE_SIZE];
  static float zAveArray[AVE_SIZE];
  static int avePtr = 0;

  float sumXAve = 0.0;
  float sumYAve = 0.0;
  float sumZAve = 0.0;

  xAveArray[avePtr] = xAve;
  yAveArray[avePtr] = yAve;
  zAveArray[avePtr] = zAve;

  avePtr = ++avePtr % AVE_SIZE;
  if (aveTotal < avePtr) aveTotal = avePtr;

  for (int i = 0; i < aveTotal; i++) {
    sumXAve += xAveArray[i];
    sumYAve += yAveArray[i];
    sumZAve += zAveArray[i];
  }
  float aveXDrift = sumXAve / aveTotal;
  float aveYDrift = sumYAve / aveTotal;
  float aveZDrift = sumZAve / aveTotal;
  timeDriftPitch = aveXDrift;
  timeDriftRoll = aveYDrift;
  timeDriftYaw = aveZDrift;
}
