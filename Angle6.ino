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
int xMin = -705;
int xMax = 585;
int yMin = -730;
int yMax = 565;
int zMin = -753;
int zMax = 296;


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
  gyroPitchRaw = (double) (gyro.g.y - pitchDrift);
//  gyroPitchRaw = (double) (gyro.g.y - gyroTempCompY); // add in constant error
  gyroPitchRate = gyroPitchRaw * GYRO_SENS;  // Rate in degreesChange/sec
  gyroPitchDelta = (gyroPitchRate * 2500.0) / 1000000.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  gaFullPitch = gyroPitchDelta + gaFullPitch;
  
  // Roll
  gyroRollRaw = -(double) (gyro.g.x + rollDrift);
//  gyroRollRaw = -(double) (gyro.g.x - gyroTempCompX);
  gyroRollRate = gyroRollRaw * GYRO_SENS;
  double gyroRollDelta = (gyroRollRate * 2500.0) / 1000000.0;
  gRoll = gRoll - gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
  gyroYawRaw = (double) (gyro.g.z - yawDrift); 
//  gyroYawRaw = (double) (gyro.g.z - gyroTempCompZ);  // subtract out constant error
//  double gyroYawRaw = (double) (gyro.g.z - 250);  // add in constant error
  //    double gyroYawRaw = (double) (gyro.g.z - ((int) (*currentValSet).y));  // add in constant error
  gyroYawRate = gyroYawRaw * GYRO_SENS;  // Rate in degreesChange/sec
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

  sumX += oldX;
  sumY += oldY;
  sumZ += oldZ;
  if ((++temperatureLoop % 400) == 0) { // Read temp every second.
    meanX = sumX / 400;
    meanY = sumY / 400;
    meanZ = sumZ / 400;
    int t = gyro.readReg(L3G::OUT_TEMP);
    if (t > 127) t -= 256;
    float temperature = (float) t;
    gyroTempCompX = 176 + ((int) (1.5 * temperature));
    gyroTempCompY = 27 + ((int) (-1.0 * temperature));
    gyroTempCompZ = 82 + ((int) (6.6 * temperature)); // 82 is normal value
    //      Serial.print(temperature); Serial.print("\t");
    //      Serial.print(gPitch); Serial.print("\t");
    //      Serial.print(gRoll); Serial.print("\t");
    //      Serial.print(gyroCumHeading); Serial.print("\t");
    //      Serial.print(meanX); Serial.print("\t");
    //      Serial.print(meanY); Serial.print("\t");
    //      Serial.print(meanZ); Serial.println();
    sumX = sumY = sumZ = 0;
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

    double mh = atan2(-headY, headX) * RAD_TO_DEG;

    if ((mh > 90.0) && (magHeading < -90.0)) { // Crossed rotating in positive direction?
      magRotations -= 360.0;
    }
    else if ((mh < -90.0) && (magHeading > 90.0)) {
      magRotations += 360.0;
    }
    magHeading = mh;
    magCumHeading = magRotations + magHeading;

    // Since this is 15/sec, we do all of our complementary filtering here

    // gmHeading. Complementary filter gyro and mag headings.
    double q = gmCumHeading + gyroCumHeading - oldGyroCumHeading;
    oldGyroCumHeading = gyroCumHeading;
    gmCumHeading = (q * GM_HEADING_TC) + (magCumHeading * (1.0 - GM_HEADING_TC));
    double c = (gmCumHeading > 0.0) ? 180.0 : -180.0;
    double rotations = (int) ((gmCumHeading + c) / 360.0);
    gmHeading = gmCumHeading - (((double) rotations) * 360.0);

    // tmHeading.  Complementary filter tick and mag headings.
    q = tmCumHeading + tickCumHeading - oldTickCumHeading;
    oldTickCumHeading = tickCumHeading;
    tmCumHeading = (q * TM_HEADING_TC) + (magCumHeading * (1.0 - TM_HEADING_TC));
    c = (tmCumHeading > 0.0) ? 180.0 : -180.0;
    rotations = (int) ((tmCumHeading + c) / 360.0);
    tmHeading = tmCumHeading - (((double) rotations) * 360.0);
  }
}

/***********************************************************************.
 *  setNavigation() Set gmHeading, tmHeading, tickHeading, currentLoc
 *                  Called 400/sec or every read of gyro.
 ***********************************************************************/
void setNavigation() {

  // Tick heading.
  tickCumHeading =  ((double) (tickHeadingOffset + tickPositionLeft - tickPositionRight)) / TICKS_PER_DEGREE_YAW;
  double c = (tickCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((tickCumHeading + c) / 360.0);
  tickHeading = tickCumHeading - (((double) rotations) * 360.0);

  // Map heading
  switch (headingSource) {
    case HEADING_SOURCE_G: // gyro
      currentMapHeading = gyroHeading;
      break;
    case HEADING_SOURCE_T: // ticks
      currentMapHeading = tickHeading;
      break;
    case HEADING_SOURCE_M: // mag
      currentMapHeading = magHeading;
      break;
    case HEADING_SOURCE_GM: // gyro & mag complementary filtered
      currentMapHeading = gmHeading;
      break;
  }
  currentMapHeading -= mapOrientation;
  if (currentMapHeading > 180.0) currentMapHeading -= 360.0;
  else if (currentMapHeading < -180.0) currentMapHeading += 360.0;
  
  // Location
 // compute the Center of Oscillation Tick Position
  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

//  double dist = ((double) (tickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  double dist = ((double) (coTickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
//  navOldTickPosition = tickPosition;
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
 * resetNavigation() Set tickHeading and gyroHeading to magHeading
 *                   when wheels touch ground or at startup.
 *                   If G or T, set mapOrientation to current orientation
 *                   If M, set to specified value.
 **************************************************************************/
void resetNavigation(char cmd, double mo) {
  gmCumHeading = tmCumHeading = gyroCumHeading = tickCumHeading = magCumHeading = magHeading;
  oldGyroCumHeading = oldTickCumHeading = magHeading;
  tickHeading = gyroHeading = magHeading;
  magRotations = 0.0;
  tickHeadingOffset = (int) (magHeading * TICKS_PER_DEGREE_YAW);
  tickPosition = tickPositionRight = tickPositionLeft = navOldTickPosition = coTickPosition = 0;
  oldTPitch = 0.0D;
  currentMapLoc.x = 0;
  currentMapLoc.y = 0;

  switch (cmd) {
    case 'M':
      mapOrientation = mo;
      headingSource = HEADING_SOURCE_GM;
      break;
    case 'T':
      mapOrientation = tickHeading;
      headingSource = HEADING_SOURCE_T;
      break;
    default:  // 'G' or error
      mapOrientation = gyroHeading;
      headingSource = HEADING_SOURCE_G;
     break; 
  }
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


//void zeroGyro() {
//  int xSum = 0;
//  int ySum = 0;
//  int zSum = 0;
//  angleInit6();
//  delay(500);
//  for (int i = 0; i < 400; i++) {
//    while (!readGyro()) {}
//    xSum += gyro.g.x;
//    ySum += gyro.g.y;
//    zSum += gyro.g.z;
//  }
//  gyroErrorX = xSum / 400;
//  gyroErrorY = ySum / 400;
//  gyroErrorZ = zSum / 400;
//  Serial.print(gyroErrorX); Serial.print("\t");
//  Serial.print(gyroErrorY); Serial.print("\t");
//  Serial.println(gyroErrorZ);
//}


