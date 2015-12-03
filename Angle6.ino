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
int xMin = -630;
int xMax = 667;
int yMin = -791;
int yMax = 509;
int zMin = -43;
int zMax = 1148;


/***********************************************************************.
 *  angleInit6()
 ***********************************************************************/
void angleInit6() {
  delay(20);
  Wire.begin();
  Wire1.begin();
  delay(20);
  compass.init(LSM303::device_D, LSM303::sa0_high);
  delay(20);
  compass.enableDefault(); // Mag: [DR=6.25 Hz, 4 gauss, contin] Accel: [2g]
  compass.writeAccReg(LSM303::CTRL1, 0x67); // Accel DR=100 HZ, all axis
  gyro.init(L3G::device_D20H, L3G::sa0_high); // Sets no parameters
  gyro.writeReg(L3G::CTRL1, 0xBF); // power, all axes, DR=400 Hz, BW=110
  gyro.writeReg(L3G::CTRL4, 0x30); // FS - 2000dps

  // Set up HMC5883L magnetometer
  Wire.beginTransmission(HC_ADDRESS);
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  delay(100);
  readCompass();  // Do this so we have a magHeading.
}

#define TG_PITCH_TC 0.90D

/***********************************************************************.
 * readGyro()
 ***********************************************************************/
boolean readGyro() {
  static int temperatureLoop = 0;

  static int oldX = 0, oldY = 0, oldZ = 0;
  int t;
  gyro.read();  // 860 microseconds
  if ((gyro.g.x == oldX) && (gyro.g.y == oldY) && (gyro.g.z == oldZ)) return false;
  oldX = gyro.g.x;
  oldY = gyro.g.y;
  oldZ = gyro.g.z;

  // Pitch
  gyroPitchRaw = ((double) gyro.g.y) - timeDriftPitch;
  gyroPitchRate = (((double) gyroPitchRaw) * GYRO_SENS) - pitchDrift;  // Rate in degreesChange/sec
  gyroPitchDelta = (gyroPitchRate * 2500.0) / 1000000.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  gaFullPitch = gyroPitchDelta + gaFullPitch;
  
  // Roll
  gyroRollRaw = (-((double) gyro.g.x)) - timeDriftRoll;
  gyroRollRate = (((double) gyroRollRaw) * GYRO_SENS) - rollDrift;
  double gyroRollDelta = (gyroRollRate * 2500.0) / 1000000.0;
  gRoll = gRoll - gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
  gyroYawRaw = ((double) gyro.g.z) - timeDriftYaw; 
//  gyroYawRaw = ((double) gyro.g.z - temperatureDriftYaw) - timeDriftYaw; 
  gyroYawRate = ((double) gyroYawRaw) * GYRO_SENS;  // Rate in degreesChange/sec
  double gyroYawDelta = (gyroYawRate * 2500.0) / 1000000.0; // degrees changed during period
  gYaw = gYaw + gyroYawDelta;
  gyroCumHeading = gyroCumHeading + gyroYawDelta;   //
  double tc = (gyroCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((gyroCumHeading + tc) / 360.0);
  gyroHeading = gyroCumHeading - (((double) rotations) * 360.0);
  isSpinExceed = (gyroYawRaw > 20000) ? true : false;

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

  sumX += gyroRollRaw;
  sumY += gyroPitchRaw;
  sumZ += gyroYawRaw;
  if ((++temperatureLoop % 400) == 0) { // Read temp every second.
    meanX = sumX / 400.0D;
    meanY = sumY / 400.0D;
    meanZ = sumZ / 400.0D;
    int t = gyro.readReg(L3G::OUT_TEMP);
    if (t > 127) t -= 256;
    t = (((33 - t) * 9) / 5) + 32;
//    gyroTempCompX = 176 + ((int) (1.5 * temperature));
//    gyroTempCompY = 27 + ((int) (-1.0 * temperature));
//    gyroTempCompZ = 82 + ((int) (6.6 * temperature)); // 82 is normal value
    //      Serial.print(temperature); Serial.print("\t");
    //      Serial.print(gPitch); Serial.print("\t");
    //      Serial.print(gRoll); Serial.print("\t");
    //      Serial.print(gyroCumHeading); Serial.print("\t");
    //      Serial.print(meanX); Serial.print("\t");
    //      Serial.print(meanY); Serial.print("\t");
    //      Serial.print(meanZ); Serial.println();
    sprintf(message, "temp: %3d  gPitchRaw: %5.2f  gRollRaw: %5.2f gYawRaw: %5.2f", t, meanY, meanX, meanZ);
//    sprintf(message, "temp: %3d  tgPitch: %5.2f  gRoll: %5.2f gYaw: %5.2f", t, gPitch, gRoll, gYaw);
//    sprintf(message, "pitchRate: %5.3f %5.3f  rollRate: %5.3f %5.3f   yawRate %5.3f %5.3f", meanY, pitchDrift, meanX, rollDrift, meanZ, yawDrift);
    sendBMsg(SEND_MESSAGE, message); 
  addLog(
        (long) (gYaw * 100.0),
        (short) (meanY * 10.0),
        (short) (meanX * 10.0),
        (short) (meanZ * 10.0),
        (short) (temperatureDriftYaw),
        (short) (timeMilliseconds/100),
        (short) t
   );
    sumX = sumY = sumZ = 0.0D;
  }

  return true;
}

/***********************************************************************.
 *  readAccel()
 ***********************************************************************/
void readAccel() {
//  if (isAccelOff) return; // If jump?
  compass.readAcc(); // 848 microseconds

  // Pitch
  double k8 = 45.5;  // for new MinImu
  //    aPitch =  (atan2(compass.a.x, -compass.a.z) * RAD_TO_DEG);
  double accelX = compass.a.x - (k8 * 1000.0 * tp6LpfCosAccel);
  aPitch = ((atan2(accelX, -compass.a.z)) * RAD_TO_DEG) + (*currentValSet).z;
  gaFullPitch = (gaFullPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  if (          ((compass.a.z > -20000) && (compass.a.z < -10000))
             && ((accelX > -7000) && (accelX < 7000))
             && ((aPitch > -45.0) && (aPitch < 45.0))) {
      gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  }

  // Roll
  aRoll =  (atan2(-compass.a.y, -compass.a.z) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}


#define GM_HEADING_TC 0.95D
//#define GM_HEADING_TC 0.995D
#define TM_HEADING_TC 0.995D

int magX, magY, magZ;

/***********************************************************************.
 *  readCompass()
 ***********************************************************************/
void readCompass() {
  double xVec, yVec, zVec;
  //  compass.readMag();
  if (readHMC()) {
    //  if ((compass.m.x != mX) || (compass.m.y != mY) || (compass.m.z != mZ)) {
    //    mX = compass.m.x;
    //    mY = compass.m.y;
    //    mZ = compass.m.z;

    // move from HMC
    mX = magX;
    mY = magY;
    mZ = magZ;


    xVec = -(((((double) (mX - xMin)) / ((double) (xMax - xMin))) * 2.0) - 1.0);
    yVec = -(((((double) (mY - yMin)) / ((double) (yMax - yMin))) * 2.0) - 1.0);
    zVec = (((((double) (mZ - zMin)) / ((double) (zMax - zMin))) * 2.0) - 1.0);
//    Serial.print(xVec); Serial.print("\t");
//    Serial.print(yVec); Serial.print("\t");
//    Serial.print(zVec); Serial.println("\t");

    headX = xVec;
    headY = yVec;

    // tilt-compenstate magnetic heading
    double gaPitchRad = gaPitch * DEG_TO_RAD;
    double gaRollRad = gaRoll * DEG_TO_RAD;
    double cosPitch = cos(gaPitchRad);
    double sinPitch = sin(gaPitchRad);
    double cosRoll = cos(gaRollRad);
    double sinRoll = sin(gaRollRad);

    headX = (xVec * cosPitch) + (yVec * sinRoll * sinPitch) + (zVec * cosRoll * sinPitch);
    headY = (yVec * cosRoll) - (zVec * sinRoll);

    magHeading = atan2(-headY, headX) * RAD_TO_DEG;
    double oldGridHeading = gridHeading;
    gridHeading = rangeAngle(magHeading - gridOffset);
    if ((oldGridHeading > 90.0D) && (gridHeading < -90.0D))  gridRotations += 360.0;
    else if ((oldGridHeading < -90.0D) && (gridHeading > 90.0D))  gridRotations -= 360.0;
    gridCumHeading = gridRotations + gridHeading;

    // Since this is 15/sec, we do all of our complementary filtering here

    // gmHeading. Complementary filter gyro and mag headings.
    double q = gmCumHeading + gyroCumHeading - oldGyroCumHeading;
    oldGyroCumHeading = gyroCumHeading;
    gmCumHeading = (q * GM_HEADING_TC) + (gridCumHeading * (1.0 - GM_HEADING_TC));
    double c = (gmCumHeading > 0.0) ? 180.0 : -180.0;
    double rotations = (int) ((gmCumHeading + c) / 360.0);
    gmHeading = gmCumHeading - (((double) rotations) * 360.0);

    // tmHeading.  Complementary filter tick and mag headings.
    q = tmCumHeading + tickCumHeading - oldTickCumHeading;
    oldTickCumHeading = tickCumHeading;
    tmCumHeading = (q * TM_HEADING_TC) + (gridCumHeading * (1.0 - TM_HEADING_TC));
    c = (tmCumHeading > 0.0) ? 180.0 : -180.0;
    rotations = (int) ((tmCumHeading + c) / 360.0);
    tmHeading = tmCumHeading - (((double) rotations) * 360.0);
  }
}

/***********************************************************************.
 *  setNavigation() Set gmHeading, tmHeading, tickHeading, currentLoc
 *                  Called 400/sec (every read of gyro).
 ***********************************************************************/
void setNavigation() {

  // Tick heading.
  tickCumHeading =  ((double) (tickPositionLeft - tickPositionRight)) / TICKS_PER_DEGREE_YAW;
  double c = (tickCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((tickCumHeading + c) / 360.0);
  tickHeading = tickCumHeading - (((double) rotations) * 360.0);

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
  currentAccelMapLoc.y += ((double) compass.a.y) * A_FACTOR;
  currentAccelMapLoc.x += ((double) compass.a.x) * A_FACTOR;
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
  oldGyroCumHeading = oldTickCumHeading = newHeading;
  tickHeading = gyroHeading = newHeading;
  gridRotations = 0.0;
  tickPosition = tickPositionRight = tickPositionLeft = navOldTickPosition = coTickPosition = 0;
  oldTPitch = 0.0D;
}


void setLoc(double x, double y) {
  currentMapLoc.x = x;
  currentMapLoc.y = y;
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
 * zeroGyro()  Take x,y, & z values for one second.  Compute the value to
 *             cancel out drift.
 **************************************************************************/
#define GLOOPS 2000  // Number of 400/sec reads
void zeroGyro() {
  int loopCount = GLOOPS;
  double sumPitch = 0;
  double sumRoll = 0;
  double sumYaw = 0;
  double gyroPitchMin, gyroPitchMax;
  double gyroRollMin, gyroRollMax;
  double gyroYawMin, gyroYawMax;
  unsigned long endTime = millis() + (GLOOPS * 3);  // Bailout time in case of bad I2C.

  temperatureDriftPitch = timeDriftPitch = 0.0D;
  temperatureDriftRoll = timeDriftRoll = 0.0D;
  temperatureDriftYaw = timeDriftYaw = 0.0D;

  for (int i = 0; i < 100; i++) {
    readGyro();
    delay(2);
  }
  
  gyroPitchMin = gyroPitchMax = gyroPitchRaw;
  gyroRollMin = gyroRollMax = gyroRollRaw;
  gyroYawMin = gyroYawMax = gyroYawRaw;
  while(true) {
    if (readGyro()) {
      sumPitch += (double) gyroPitchRaw;
      if (gyroPitchRaw > gyroPitchMax) gyroPitchMax = gyroPitchRaw;
      if (gyroPitchRaw < gyroPitchMin) gyroPitchMin = gyroPitchRaw;
      sumRoll += (double) gyroRollRaw;
      if (gyroRollRaw > gyroRollMax) gyroRollMax = gyroRollRaw;
      if (gyroRollRaw < gyroRollMin) gyroRollMin = gyroRollRaw;
      sumYaw += (double) gyroYawRaw;
      if (gyroYawRaw > gyroYawMax) gyroYawMax = gyroYawRaw;
      if (gyroYawRaw < gyroYawMin) gyroYawMin = gyroYawRaw;
      if (--loopCount <= 0) break;
      if (millis() > endTime) break;
    }
  }
  if (     ((gyroPitchMax - gyroPitchMin) < 50)
        && ((gyroRollMax - gyroRollMin) < 50)
        && ((gyroYawMax - gyroYawMin) < 50)) {
    timeDriftPitch = (sumPitch / ((double) GLOOPS));
    timeDriftRoll = (sumRoll / ((double) GLOOPS));
    timeDriftYaw = (sumYaw / ((double) GLOOPS));
    beep(BEEP_UP);
  } else {
    timeDriftPitch = -9.55;
    timeDriftRoll = -25.27;
    timeDriftYaw = -8.59;
    beep(BEEP_DOWN);
  }

  Serial.println();
  Serial.print("pitchDrift: "); Serial.print(timeDriftPitch); Serial.print("\t");
  Serial.print("rollDrift: "); Serial.print(timeDriftRoll); Serial.print("\t");
  Serial.print("yawDrift: "); Serial.print(timeDriftYaw); Serial.println();
  
  Serial.print("Pitch min: "); Serial.print(gyroPitchMin,0); Serial.print("\t");
  Serial.print("Pitch Max: "); Serial.print(gyroPitchMax,0); Serial.print("\t");
  Serial.print("Pitch Range: "); Serial.print(gyroPitchMax - gyroPitchMin,0); Serial.println();
  
  Serial.print("Roll min: "); Serial.print(gyroRollMin,0); Serial.print("\t");
  Serial.print("Roll Max: "); Serial.print(gyroRollMax,0); Serial.print("\t");
  Serial.print("Roll Range: "); Serial.print(gyroRollMax - gyroRollMin,0); Serial.println();
  
  Serial.print("Yaw min: "); Serial.print(gyroYawMin,0); Serial.print("\t");
  Serial.print("Yaw Max: "); Serial.print(gyroYawMax,0); Serial.print("\t");
  Serial.print("Yaw Range: "); Serial.print(gyroYawMax - gyroYawMin,0); Serial.println();
  
  gPitch = 0.0;
  gRoll = 0.0;
  gYaw = 0.0;
  int t = gyro.readReg(L3G::OUT_TEMP);
  if (t > 127) t -= 256;
  gyroFahrenheit = (((33 - t) * 9) / 5) + 32;
  Serial.print("Temperature: "); Serial.println(gyroFahrenheit);
}

