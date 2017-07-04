/***********************************************************************.
 *  Angle6
 ***********************************************************************/
#define HC_ADDRESS 0x1E


// LSM303 compass
//int xMin = -2621;  // -953;   // -1107; // -1060;
//int xMax = 4029;   // 6187;   // 5315; //  5507;
//int yMin = -3180;  // -2924;  // -3284; // -3582;
//int yMax = 3022;   // 4130;   // 2997; //  2764;
//int zMin = -3600;  // -3893;  // -2325; // -2070;
//int zMax = 2698;   // 2819;   // 3702; //  4216;
//

//HMC5883L compass
const int xMin = -630;
const int xMax = 667;
const int yMin = -791;
const int yMax = 509;
const int zMin = -43;
const int zMax = 1148;

int zSum = 0;
int zCount = 0;
int yawTempComp = 0;
/***********************************************************************.
 *  angleInit6()
 ***********************************************************************/
void angleInit6() {
  int success;
  Wire.begin();
  for (int i = 0; i < 100; i++) {
    success = lsm6.init(); 
    if (success) break; 
    Serial.println("IMU initialize failed!");
    resetIMU();
    Wire.begin();
  }
  if (success) Serial.println("IMU Initialized!****************************");
  lsm6.enableDefault();
  lsm6.writeReg(LSM6::INT1_CTRL, 0X02); // Accel data ready on INT1
  lsm6.writeReg(LSM6::INT2_CTRL, 0X01); // Gyro data ready on INT2
  lsm6.writeReg(LSM6::CTRL2_G, 0X5C);   // Gyro 2000fs, 208hz
  lsm6.writeReg(LSM6::CTRL1_XL, 0X50);  // Accel 2g, 208hz
//  lsm6.writeReg(LSM6::CTRL2_G, 0X6C);   // Gyro 2000fs, 416hz
//  lsm6.writeReg(LSM6::CTRL1_XL, 0X40);  // Accel 2g, 104hz

  // Set up HMC5883L magnetometer
//  Wire.beginTransmission(HC_ADDRESS);
//  Wire.write(0x02); //select mode register
//  Wire.write(0x00); //continuous measurement mode
//  Wire.endTransmission();
//
//  delay(100);
//  readCompass();  // Do this so we have a magHeading.
}

#define TG_PITCH_TC 0.90D

/***********************************************************************.
 * setGyroData()
 ***********************************************************************/
void setGyroData() {
  int t;
  // Pitch
  gyroPitchRaw = ((double) lsm6.g.x) - timeDriftPitch;
  gyroPitchRate = (((double) gyroPitchRaw) * GYRO_SENS);  // Rate in degreesChange/sec
  gyroPitchDelta = -gyroPitchRate / 208.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  gaFullPitch = gyroPitchDelta + gaFullPitch;
  
  // Roll
  gyroRollRaw = ((double) lsm6.g.y) - timeDriftRoll;
  gyroRollRate = (((double) gyroRollRaw) * GYRO_SENS);
  double gyroRollDelta = gyroRollRate / 208.0;
  gRoll = gRoll - gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
//  gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw - temperatureDriftYaw; 
//sumYaw += lsm6.g.z;
  gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw; 
  gyroYawRate = ((double) gyroYawRaw) * GYRO_SENS;  // Rate in degreesChange/sec
  double gyroYawDelta = -gyroYawRate / 208.0; // degrees changed during period
  gYaw += gyroYawDelta;
  gyroCumHeading += gyroYawDelta;   //
  double tc = (gyroCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((gyroCumHeading + tc) / 360.0);
  gyroHeading = gyroCumHeading - (((double) rotations) * 360.0);

  // Yaw for gmHeading
  gmCumHeading += gyroYawDelta;
  tc = (gmCumHeading > 0.0) ? 180 : - 180;
  rotations = (int) ((gmCumHeading + tc) / 360.0);
  gmHeading = gmCumHeading - (((double) rotations) * 360.0);

  // Rotation rate: complementary filtered gPitch and tPitch
  tPitch = (double) tickPosition / TICKS_PER_PITCH_DEGREE;
  double qDiff = tPitch - oldTPitch;
  double q = tgPitch + qDiff;
  oldTPitch = tPitch;
  tgPitch = (q * TG_PITCH_TC) + (gPitch * (1.0D - TG_PITCH_TC));
  tgPitchDelta = tgPitch - oldTgPitch;  // Used in Algorithm ***************
  oldTgPitch = tgPitch;

  // tgaPitch:
  q = tgaPitch + qDiff;
  tgaPitch = (q * TG_PITCH_TC) + (gaPitch * (1.0D - TG_PITCH_TC));
}

/***********************************************************************.
 *  setAccelData()
 ***********************************************************************/
void setAccelData() {
  static int lastAccel;
  // Pitch
//  double k8 = 2.0;  // 
  double k8 = (*currentValSet).v;
  double accelX = lsm6.a.y + (k8 * 100000.0 * lpfCos3Accel);  // 
//  double accelX = lsm6.a.y;  // 
  aPitch = ((atan2(-accelX, lsm6.a.z)) * RAD_TO_DEG) + (*currentValSet).z;
  gaFullPitch = (gaFullPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  if ((          (lsm6.a.z > 7000)
             && ((accelX > -7000) && (accelX < 7000))
             && ((aPitch > -45.0) && (aPitch < 45.0))) || !isRunning) {
      gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
    }

  // y accel
  yAccel += lastAccel - lsm6.a.y;
  lastAccel = lsm6.a.y;
  
  // Roll
  aRoll =  (atan2(lsm6.a.x, lsm6.a.z) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}


//#define GM_HEADING_TC 0.95D
#define GM_HEADING_TC 0.98D
#define TM_HEADING_TC 0.999D

int magX, magY, magZ;
/***********************************************************************.
 *  readTemperature()  Read the gyro temperature and set the yawTempComp
 *                     variable 
 ***********************************************************************/
//#define YAW_TEMP_FACTOR 0.84  // 
//void readTemperature() {
//  int t = gyro.readReg(L3G::OUT_TEMP);
//  if (t > 127) t -= 256;
//  gyroFahrenheit = (((33 - t) * 9) / 5) + 32;
//  yawTempComp = (int) (((float) (startTemp - t)) * YAW_TEMP_FACTOR);
//}


 
/***********************************************************************.
 *  setNavigation() Set gmHeading, tmHeading, tickHeading, currentLoc
 *                  Called 400/sec (every read of gyro).
 ***********************************************************************/
#define TICK_BIAS_TRIGGER 500
void setNavigation() {
  
  tickPosition = tickPositionRight + tickPositionLeft;

  // Tick bias
//  if ((tickPosition - lastTickSet) > TICK_BIAS_TRIGGER) {
//    lastTickSet = tickPosition;
//    tickPositionRight++;
//    tickPositionLeft--;
//  }
//  if ((lastTickSet - tickPosition) > TICK_BIAS_TRIGGER) {
//    lastTickSet = tickPosition;
//    tickPositionRight--;
//    tickPositionLeft++;
//  }


  // Tick heading.
  tickCumHeading =  ((double) (tickOffset + tickPositionLeft - tickPositionRight)) / TICKS_PER_DEGREE_YAW;
  double c = (tickCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((tickCumHeading + c) / 360.0); 
  tickHeading = tickCumHeading - (((double) rotations) * 360.0);
  
  // tmHeading.  Complementary filter tick and mag headings.
  tmCumHeading += tickCumHeading - oldTickCumHeading;
  oldTickCumHeading = tickCumHeading;
  tmCumHeading = (tmCumHeading * TM_HEADING_TC) + (gridCumHeading * (1.0 - TM_HEADING_TC));
  c = (tmCumHeading > 0.0) ? 180.0 : -180.0;
  rotations = (int) ((tmCumHeading + c) / 360.0);
  tmHeading = tmCumHeading - (((double) rotations) * 360.0);

 // compute the Center of Oscillation Tick Position
  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

  // Compute the new co position
  double dist = ((double) (coTickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  navOldTickPosition = coTickPosition;
  currentLoc.x += sin(gyroHeading * DEG_TO_RAD) * dist;
  currentLoc.y += cos(gyroHeading * DEG_TO_RAD) * dist;

  currentAccelLoc();

  
}




//double accelFpsSelfX = 0.0;
//double accelFpsSelfY = 0.0;
//double accelFpsMapX = 0.0;
//double accelFpsMapY = 0.0;
//struct loc currentAccelSelfLoc;
//struct loc currentAccelMapLoc;

const double A_FACTOR = .000001D;
/***********************************************************************.
 *  setAccelLoc() Set currentAccelLoc
 ***********************************************************************/
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



/**************************************************************************.
 * setHeading() Sets the bearing to the new value.  The the gridOffset
 *              value will be set so that the gridBearing is an
 *              offset from magHeading.  All of cumulative rotations 
 *              be lost.
 **************************************************************************/
void setHeading(double newHeading) {
  gmCumHeading = tmCumHeading = gyroCumHeading = tickCumHeading = gridCumHeading = newHeading;
  tickOffset = newHeading * TICKS_PER_DEGREE_YAW;
  oldGyroCumHeading = oldTickCumHeading = newHeading;
  tickHeading = gyroHeading = newHeading;
//  lastTickSet = 0;
  gridRotations = 0.0;
}

void resetTicks() {
  tickPosition = tickPositionRight = tickPositionLeft = navOldTickPosition = coTickPosition = 0;
  oldTPitch = 0.0D;
}


/**************************************************************************.
 * readHMC() Routine taken from CalibrateHMC
 **************************************************************************/
boolean readHMC() {
  int xab, yab, zab;
  unsigned int timeOut = 2;

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(HC_ADDRESS);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  Wire.requestFrom(HC_ADDRESS, 6);

  unsigned int millisStart = millis();

  //Read data from each axis, 2 registers per axis
  while (Wire.available() < 6) {
    if ((timeOut > 0) && ((millis() - millisStart) > timeOut)) {
      return false;
    }
  }

  byte xhm = Wire.read(); //X msb
  byte xlm = Wire.read(); //X lsb
  byte zhm = Wire.read(); //Z msb
  byte zlm = Wire.read(); //Z lsb
  byte yhm = Wire.read(); //Y msb
  byte ylm = Wire.read(); //Y lsb

  xab = (int16_t)(xhm << 8 | xlm);
  yab = (int16_t)(yhm << 8 | ylm);
  zab = (int16_t)(zhm << 8 | zlm);

  if ((xab != magX) || (yab != magY) || (zab != magZ)) {
    magX = xab;
    magY = yab;
    magZ = zab;
    return true;
  }
  return false;
}



/**************************************************************************.
 * isNew???()  Return true if new data has been read.
 **************************************************************************/
boolean isNewGyro() {
  if (digitalRead(GYRO_INTR_PIN) == LOW) return false;
  lsm6.readGyro();
  return true;
}
boolean isNewAccel() {
  if (digitalRead(ACCEL_INTR_PIN) == LOW) return false;
  lsm6.readAcc();
  return true;
}




/**************************************************************************.
 * resetIMU()  From: https://forum.arduino.cc/index.php?topic=386269.0
 *             I2C clocks to make sure no slaves are hung in a read
 *             at startup
 **************************************************************************/
void resetIMU() {
  // Issue 20 I2C clocks to make sure no slaves are hung in a read
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(70, OUTPUT);
  pinMode(71, OUTPUT);
  digitalWrite(20, LOW);
  digitalWrite(70, LOW);
  for (int i = 0; i < 1000; i++)
  {
    digitalWrite(21, LOW);
    digitalWrite(71, LOW);
    delayMicroseconds(10);
    digitalWrite(21, HIGH);
    digitalWrite(71, HIGH);
    delayMicroseconds(10);
  }
}



/**************************************************************************.
 * readFahr()  Read the Fahrenheit temperature from the gyro
 **************************************************************************/
float readFahr() {
  Wire.beginTransmission(0b1101011);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(LSM6::OUT_TEMP_L);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) 0b1101011, (uint8_t) 2);

  while (Wire.available() < 2) {
  }

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  // combine high and low bytes
  int16_t out = (int16_t)(th << 8 | tl);
  float ret = ((float) out) / 16;
  ret += 25.0;
  ret = ((ret * 9.0) / 5) + 32;
  return ret;
}



const int DRIFT_SIZE = 204;
float zDriftSum = 0.0;
float xDriftSum = 0.0;
int gDriftCount = 0;
int gPtr = 0;
boolean isMeasuringDrift = false;
/**************************************************************************.
 * doGyroDrift()  Called 204/sec.  Average gyroDrift for x, y & z
 *                for one second periods.
 **************************************************************************/
void doGyroDrift() {
  static int zArray[DRIFT_SIZE];
  static int xArray[DRIFT_SIZE];
  int z, x;

  if (!isMeasuringDrift) return;
  zArray[gPtr] = lsm6.g.z;
  xArray[gPtr] = lsm6.g.x;
  gPtr++;
  if (gPtr >= DRIFT_SIZE) {
    int zSum = 0;
    int xSum = 0;
    int xMax = xArray[0];
    int xMin = xMax;
    int zMax = zArray[0];
    int zMin = zMax;
    for (int i = 0; i < DRIFT_SIZE; i++) {
      z = zArray[i];
      if (z > zMax)  zMax = z;
      if (z < zMin)  zMin = z;
      zSum += z;
      x = xArray[i];
      if (x > xMax)  xMax = x;
      if (x < xMin)  xMin = x;
      xSum += x;
    }
    float zAve = ((float) zSum) / ((float) DRIFT_SIZE);
    float xAve = ((float) xSum) / ((float) DRIFT_SIZE);
    if (((zMax - zMin) < 30) && ((xMax - xMin) < 30)) {
      zDriftSum += zAve;
      xDriftSum += xAve;
      gDriftCount++;
    }
//    sprintf(message, "xMin: %4d     xMax: %4d     xAve: %5.1f   ", xMax, xMin, xAve);
//    Serial.print(message);
//    sprintf(message, "zMin: %4d     zMax: %4d     zAve: %5.1f\n", zMax, zMin, zAve);
//    Serial.print(message);
    gPtr = 0;
  }
}



/**************************************************************************.
 * startGyroDrift()  "isRunning" has changed to false.  Initialize the
 *                   sums for x, y, and z.
 **************************************************************************/
void startGyroDrift() {
  isMeasuringDrift = true;
  gPtr = 0;
  xDriftSum = zDriftSum = 0.0;
  gDriftCount = 0;
}



/**************************************************************************.
 * setGyroDrift()  "isRunning" has changed to true.  Take the averaged 
 *                 drift values and set the "timeDrift??? values.
 **************************************************************************/
void setGyroDrift() {
  if (gDriftCount > 0) {
    timeDriftYaw = zDriftSum / ((float) gDriftCount);
    timeDriftPitch = xDriftSum / ((float) gDriftCount);
    sprintf(message, "YawDrift: %5.2f     PitchDrift: %5.2f\n", timeDriftYaw, timeDriftPitch);
    sendBMsg(SEND_MESSAGE, message);
    Serial.print(message);
  }
  isMeasuringDrift = false;
}





 

