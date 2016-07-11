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
  Wire.begin();
  if (!lsm6.init()) Serial.println("IMU initialize failed!");
  else Serial.println("IMU Initialized!****************************");
  lsm6.enableDefault();
  lsm6.writeReg(LSM6::INT1_CTRL, 0X02); // Gyro data on INT1
  lsm6.writeReg(LSM6::INT2_CTRL, 0X01); // Gyro data ready on INT2
  lsm6.writeReg(LSM6::CTRL2_G, 0X6C);   // Gyro 2000fs, 416hz
  lsm6.writeReg(LSM6::CTRL1_XL, 0X40);  // Accel 2g, 104hz

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
gSet++;
  // Pitch
  gyroPitchRaw = ((double) lsm6.g.x) - timeDriftPitch;
  gyroPitchRate = (((double) gyroPitchRaw) * GYRO_SENS);  // Rate in degreesChange/sec
  gyroPitchDelta = -gyroPitchRate / 416.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  gaFullPitch = gyroPitchDelta + gaFullPitch;
  
  // Roll
  gyroRollRaw = ((double) lsm6.g.y) - timeDriftRoll;
  gyroRollRate = (((double) gyroRollRaw) * GYRO_SENS);
  double gyroRollDelta = gyroRollRate / 416.0;
  gRoll = gRoll - gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
//  gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw - temperatureDriftYaw; 
sumYaw += lsm6.g.z;
  gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw; 
  gyroYawRate = ((double) gyroYawRaw) * GYRO_SENS;  // Rate in degreesChange/sec
  double gyroYawDelta = -gyroYawRate / 416.0; // degrees changed during period
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
  tgPitchDelta = tgPitch - oldTgPitch;
  oldTgPitch = tgPitch;

  // tgaPitch:
  q = tgaPitch + qDiff;
  tgaPitch = (q * TG_PITCH_TC) + (gaPitch * (1.0D - TG_PITCH_TC));
}

/***********************************************************************.
 *  setAccelData()
 ***********************************************************************/
void setAccelData() {

  // Pitch
  double k8 = 45.5;  // for new MinImu
  double accelX = lsm6.a.y + (k8 * 1000.0 * tp6LpfCosAccel);
  aPitch = ((atan2(-accelX, lsm6.a.z)) * RAD_TO_DEG) + (*currentValSet).z;
  gaFullPitch = (gaFullPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  if (          (lsm6.a.z > 7000)
             && ((accelX > -7000) && (accelX < 7000))
             && ((aPitch > -45.0) && (aPitch < 45.0))) {
      gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
    }

  // Roll
  aRoll =  (atan2(lsm6.a.x, lsm6.a.z) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}


//#define GM_HEADING_TC 0.95D
#define GM_HEADING_TC 0.98D
#define TM_HEADING_TC 0.999D

int magX, magY, magZ;

/***********************************************************************.
 *  readCompass()
 ***********************************************************************/
//void readCompass() {
////  double xVec, yVec, zVec;
//  //  compass.readMag();
//  if (readHMC()) {
//    //  if ((compass.m.x != mX) || (compass.m.y != mY) || (compass.m.z != mZ)) {
//    //    mX = compass.m.x;
//    //    mY = compass.m.y;
//    //    mZ = compass.m.z;
//
//    // move from HMC
//    mX = magX;
//    mY = magY;
//    mZ = magZ;
//
//
//    xVec = -(((((double) (mX - xMin)) / ((double) (xMax - xMin))) * 2.0) - 1.0);
//    yVec = -(((((double) (mY - yMin)) / ((double) (yMax - yMin))) * 2.0) - 1.0);
//    zVec = (((((double) (mZ - zMin)) / ((double) (zMax - zMin))) * 2.0) - 1.0);
////    Serial.print(xVec); Serial.print("\t");
////    Serial.print(yVec); Serial.print("\t");
////    Serial.print(zVec); Serial.println("\t");
//
//    headX = xVec;
//    headY = yVec;
//
//    // tilt-compenstate magnetic heading
//    double gaPitchRad = gaPitch * DEG_TO_RAD;
//    double gaRollRad = gaRoll * DEG_TO_RAD;
//    double cosPitch = cos(gaPitchRad);
//    double sinPitch = sin(gaPitchRad);
//    double cosRoll = cos(gaRollRad);
//    double sinRoll = sin(gaRollRad);
//
//    headX = (xVec * cosPitch) + (yVec * sinRoll * sinPitch) + (zVec * cosRoll * sinPitch);
//    headY = (yVec * cosRoll) - (zVec * sinRoll);
//
//    magHeading = atan2(-headY, headX) * RAD_TO_DEG;
//    double oldGridHeading = gridHeading;
//    gridHeading = rangeAngle(magHeading - gridOffset);
//    if ((oldGridHeading > 90.0D) && (gridHeading < -90.0D))  gridRotations += 360.0;
//    else if ((oldGridHeading < -90.0D) && (gridHeading > 90.0D))  gridRotations -= 360.0;
//    gridCumHeading = gridRotations + gridHeading;
//
//    // Since this is 15/sec, we do all of our complementary filtering here
//
//    // gmHeading. Complementary filter gyro and mag headings.
//    gmCumHeading = (gmCumHeading * GM_HEADING_TC) + (gridCumHeading * (1.0 - GM_HEADING_TC));
//    double c = (gmCumHeading > 0.0) ? 180.0 : -180.0;
//    double rotations = (int) ((gmCumHeading + c) / 360.0);
//    gmHeading = gmCumHeading - (((double) rotations) * 360.0);
//  }
//}



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
  if ((tickPosition - lastTickSet) > TICK_BIAS_TRIGGER) {
    lastTickSet = tickPosition;
    tickPositionRight++;
    tickPositionLeft--;
  }
  if ((lastTickSet - tickPosition) > TICK_BIAS_TRIGGER) {
    lastTickSet = tickPosition;
    tickPositionRight--;
    tickPositionLeft++;
  }


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

  // Map heading
  switch (headingSource) {
    case HEADING_SOURCE_G: // gyro
      currentMapCumHeading = gyroCumHeading;
      currentMapHeading = gyroHeading;
      break;
    case HEADING_SOURCE_T: // ticks
      currentMapCumHeading = tickCumHeading;
      currentMapHeading = tickHeading;
      break;
    case HEADING_SOURCE_M: // mag
      currentMapCumHeading = gridCumHeading;
      currentMapHeading = magHeading;
      break;
    case HEADING_SOURCE_GM: // gyro & mag complementary filtered
      currentMapCumHeading = gmCumHeading;
      currentMapHeading = gmHeading;
      break;
  }
 
 // compute the Center of Oscillation Tick Position
  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

  // Compute the new co position
  double dist = ((double) (coTickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  navOldTickPosition = coTickPosition;
  currentMapLoc.x += sin(currentMapHeading * DEG_TO_RAD) * dist;
  currentMapLoc.y += cos(currentMapHeading * DEG_TO_RAD) * dist;

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
  lastTickSet = 0;
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
 * zeroGyro()  Take x,y, & z values for ? second.  Compute the value to
 *             cancel out drift.
 **************************************************************************/
#define GLOOPS 2000  // Number of 400/sec reads
void zeroGyro() {
  int loopCount = GLOOPS;
  int sumPitch = 0;
  int sumRoll = 0;
  int sumYaw = 0;
  int gyroPitchMin, gyroPitchMax;
  int gyroRollMin, gyroRollMax;
  int gyroYawMin, gyroYawMax;

  for (int i = 0; i < 100; i++) {
    lsm6.readGyro();
    delay(3);
  }
  
  gyroPitchMin = gyroPitchMax = lsm6.g.x;
  gyroRollMin = gyroRollMax = lsm6.g.y;
  gyroYawMin = gyroYawMax = lsm6.g.z;
  while(true) {
    if (isNewGyro()) {
      int pitch = lsm6.g.x;
      sumPitch += pitch;
      if (pitch > gyroPitchMax) gyroPitchMax = pitch;
      if (pitch < gyroPitchMin) gyroPitchMin = pitch;
      
      int roll = lsm6.g.y;
      sumRoll += roll;
      if (roll > gyroRollMax) gyroRollMax = roll;
      if (roll < gyroRollMin) gyroRollMin = roll;

      int yaw = lsm6.g.z;
      sumYaw += yaw;
      if (yaw > gyroYawMax) gyroYawMax = yaw;
      if (yaw < gyroYawMin) gyroYawMin = yaw;
      if (--loopCount <= 0) break;
    }
  }
  
  timeDriftPitch = (((double) sumPitch) / ((double) GLOOPS)) - 0.80; //************* FUDGE! ****************
  timeDriftRoll = (((double) sumRoll) / ((double) GLOOPS)) + 1.50 ; //************* FUDGE! ****************
  timeDriftYaw = (((double) sumYaw) / ((double) GLOOPS)) + 0.20; //************* FUDGE! ****************
  if (     ((gyroPitchMax - gyroPitchMin) < 20)
        && ((gyroRollMax - gyroRollMin) < 20)
        && ((gyroYawMax - gyroYawMin) < 20)) {
    beep(BEEP_UP);
  } else {
    timeDriftPitch = PITCH_DRIFT;
    timeDriftRoll = ROLL_DRIFT;
    timeDriftYaw = YAW_DRIFT;
    beep(BEEP_DOWN);
  }

  Serial.println();
  Serial.print("pitchDrift: "); Serial.print(timeDriftPitch); Serial.print("\t");
  Serial.print("rollDrift: "); Serial.print(timeDriftRoll); Serial.print("\t");
  Serial.print("yawDrift: "); Serial.print(timeDriftYaw); Serial.println();
  
  Serial.print("Pitch min: "); Serial.print(gyroPitchMin); Serial.print("\t");
  Serial.print("Pitch Max: "); Serial.print(gyroPitchMax); Serial.print("\t");
  Serial.print("Pitch Range: "); Serial.print(gyroPitchMax - gyroPitchMin); Serial.println();
  
  Serial.print("Roll min: "); Serial.print(gyroRollMin); Serial.print("\t");
  Serial.print("Roll Max: "); Serial.print(gyroRollMax); Serial.print("\t");
  Serial.print("Roll Range: "); Serial.print(gyroRollMax - gyroRollMin); Serial.println();
  
  Serial.print("Yaw min: "); Serial.print(gyroYawMin); Serial.print("\t");
  Serial.print("Yaw Max: "); Serial.print(gyroYawMax); Serial.print("\t");
  Serial.print("Yaw Range: "); Serial.print(gyroYawMax - gyroYawMin); Serial.println();
  
  baseGyroTemp = readFahr();
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
