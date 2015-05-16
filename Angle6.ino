/***********************************************************************.
 *  Angle6 
 ***********************************************************************/
L3G gyro;
LSM303 compass;

int xMin = -953;   // -1107; // -1060;
int xMax = 6187;   // 5315; //  5507;
int yMin = -2924;  // -3284; // -3582;
int yMax = 4130;   // 2997; //  2764;
int zMin = -3893;  // -2325; // -2070;
int zMax = 2819;   // 3702; //  4216;

void angleInit6() {
  Wire.begin();
  compass.init();
  compass.enableDefault(); // Mag: [DR=6.25 Hz, 4 gauss, contin] Accel: [2g]
  compass.writeAccReg(LSM303::CTRL1, 0x67); // Accel DR=100 HZ, all axis
  gyro.init(); // Sets no parameters
  gyro.writeReg(L3G::CTRL1, 0xBF); // power, all axes, DR=400 Hz, BW=110
}

#define TG_PITCH_TC 0.90D

boolean readGyro() {
  static int oldX, oldY, oldZ;
    gyro.read();  // 860 microseconds
    if ((gyro.g.x == oldX) && (gyro.g.y == oldY) && (gyro.g.z == oldZ)) return false;
    oldX = gyro.g.x;
    oldY = gyro.g.y;
    oldZ = gyro.g.z;
    
    // Pitch
    double gyroPitchRaw = (double) (gyro.g.y - gyroErrorX); // add in constant error
    double gyroPitchRate = gyroPitchRaw * GYRO_SENS;  // Rate in degreesChange/sec
    gyroPitchDelta = (gyroPitchRate * 2500.0) / 1000000.0; // degrees changed during period
    gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
    gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
    
    // Roll
    double gyroRollRaw = -(double) (gyro.g.x - gyroErrorY); 
    double gyroRollRate = gyroRollRaw * GYRO_SENS;
    double gyroRollDelta = (gyroRollRate * 2500.0) / 1000000.0;
    gRoll = gRoll + gyroRollDelta;
    gaRoll = gyroRollDelta + gaRoll;
    
    // Yaw
    double gyroYawRaw = (double) (gyro.g.z - gyroErrorZ);  // add in constant error
    double gyroYawRate = gyroYawRaw * GYRO_SENS;  // Rate in degreesChange/sec
    double gyroYawDelta = (gyroYawRate * 2500.0) / 1000000.0; // degrees changed during period
    gyroCumHeading = gyroCumHeading + gyroYawDelta;   //
    double tc = (gyroCumHeading > 0.0) ? 180.0 : -180.0;
    int rotations = (int) ((gyroCumHeading + tc) / 360.0);
    gyroHeading = gyroCumHeading - (((double) rotations) * 360.0);
    
    // Do the tick angle and the comp filtered gtPitch to add the tick angle
    tPitch = (double) tickPosition / TICKS_PER_PITCH_DEGREE;
    double q = tgPitch + (tPitch - oldTPitch);
    oldTPitch = tPitch;
    tgPitch = (q * TG_PITCH_TC) + (gPitch * (1.0D - TG_PITCH_TC));
    tgPitchDelta = tgPitch - oldTgPitch;
    oldTgPitch = tgPitch;
    
    return true;
}

boolean readAccel() {
    compass.readAcc(); // 848 microseconds
    
    // Pitch
    double k8 = 45.5;  // for new MinImu
//    aPitch =  (atan2(compass.a.x, -compass.a.z) * RAD_TO_DEG);
    aPitch = ((atan2((compass.a.x - (k8 * 1000.0 * tp6LpfCosAccel)), -compass.a.z)) * RAD_TO_DEG) + (*currentValSet).z;
    gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT)); // Weigh factors
    
    // Roll
    aRoll =  (atan2(-compass.a.y, -compass.a.z) * RAD_TO_DEG);
    gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors 
}


#define GM_HEADING_TC 0.995D
#define TM_HEADING_TC 0.995D

void readCompass() {
  double xVec, yVec, zVec;
  compass.readMag();
  if ((compass.m.x != mX) || (compass.m.y != mY) || (compass.m.z != mZ)) {
    mX = compass.m.x;
    mY = compass.m.y;
    mZ = compass.m.z;
 
    xVec = ((((double) (mX - xMin))/((double) (xMax - xMin))) * 2.0) - 1.0;
    yVec = ((((double) (mY - yMin))/((double) (yMax - yMin))) * 2.0) - 1.0;
    zVec = ((((double) (mZ - zMin))/((double) (zMax - zMin))) * 2.0) - 1.0;
    
    headX = xVec;
    headY = yVec;

    // tilt-compenstated magnetic heading
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
    
    // Since this is the slowest, we do all of our complementary filtering here
  
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
 ***********************************************************************/
void setNavigation() {
  
  // Tick heading.
  tickCumHeading =  ((double) (tickHeadingOffset + tickPositionLeft - tickPositionRight)) / TICKS_PER_DEGREE_YAW;
  double c = (tickCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((tickCumHeading + c) / 360.0);
  tickHeading = tickCumHeading - (((double) rotations) * 360.0);
  
    // Set the current map heading
  double h = gyroHeading - mapOrientation;  // ---------------- Set source for map heading------------------------
  if (h > 180.0) h -= 360.0;
  else if (h < -180.0) h += 360.0;
  currentMapHeading = h;
  
  // Set the current location
  double dist = ((double) (tickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  navOldTickPosition = tickPosition;
  currentMapLoc.x += sin(currentMapHeading * DEG_TO_RAD) * dist;
  currentMapLoc.y += cos(currentMapHeading * DEG_TO_RAD) * dist;
}


/**************************************************************************.
 * resetNavigation() Set tickHeading and gyroHeading to magHeading 
 *                   when wheels touch ground or at startup.
 **************************************************************************/
void resetNavigation() {
  gmCumHeading = tmCumHeading = gyroCumHeading = tickCumHeading = magCumHeading = magHeading;
  oldGyroCumHeading = oldTickCumHeading = magHeading;
  tickHeading = gyroHeading = magHeading;
  magRotations = 0.0;
  tickHeadingOffset = (int) (magCumHeading * TICKS_PER_DEGREE_YAW);
  tickPosition = tickPositionRight = tickPositionLeft = navOldTickPosition = 0;
  oldTPitch = 0.0D;
  currentMapLoc.x = 0;
  currentMapLoc.y = 0;
  mapOrientation = DEFAULT_MAP_ORIENTATION;
}

void zeroGyro() {
  int xSum = 0;
  int ySum = 0;
  int zSum = 0;
  angleInit6();
  delay(500);
  for (int i = 0; i < 400; i++) {
    while (!readGyro()) {}
    xSum += gyro.g.x;
    ySum += gyro.g.y;
    zSum += gyro.g.z;
  }
  gyroErrorX = xSum / 400;
  gyroErrorY = ySum / 400;
  gyroErrorZ = zSum / 400;
  Serial.print(gyroErrorX); Serial.print("\t");
  Serial.print(gyroErrorY); Serial.print("\t");
  Serial.println(gyroErrorZ); 
}


