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

void readGyro() {
    gyro.read();  // 860 microseconds
    
    // Pitch
    float gyroPitchRaw = (float) (gyro.g.y - 290); // add in constant error
    float gyroPitchRate = gyroPitchRaw * GYRO_SENS;  // Rate in degreesChange/sec
    gyroPitchDelta = (gyroPitchRate * 2500.0) / 1000000.0; // degrees changed during period
    gPitch = gPitch + gyroPitchDelta;   // Not used.  Only for debugging purposes
    gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
    
    // Roll
    float gyroRollRaw = -(float) (gyro.g.x - 52); 
    float gyroRollRate = gyroRollRaw * GYRO_SENS;
    float gyroRollDelta = (gyroRollRate * 2500.0) / 1000000.0;
    gRoll = gRoll + gyroRollDelta;
    gaRoll = gyroRollDelta + gaRoll;
    
    // Yaw
    float gyroYawRaw = (float) (gyro.g.z - 165);  // add in constant error
    float gyroYawRate = gyroYawRaw * GYRO_SENS;  // Rate in degreesChange/sec
    float gyroYawDelta = (gyroYawRate * 2500.0) / 1000000.0; // degrees changed during period
    gyroCumHeading = gyroCumHeading + gyroYawDelta;   //
    float tc = (gyroCumHeading > 0.0) ? 180.0 : -180.0;
    int rotations = (int) ((gyroCumHeading + tc) / 360.0);
    gyroHeading = gyroCumHeading - (((float) rotations) * 360.0);
}

boolean readAccel() {
    compass.readAcc(); // 848 microseconds
    
    // Pitch
    float k8 = 45.5;  // for new MinImu
//    aPitch =  (atan2(compass.a.x, -compass.a.z) * RAD_TO_DEG);
    aPitch = ((atan2((compass.a.x - (k8 * 1000.0 * tp6LpfCosAccel)), -compass.a.z)) * RAD_TO_DEG) + (*currentValSet).z;
    gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT)); // Weigh factors
    
    // Roll
    aRoll =  (atan2(-compass.a.y, -compass.a.z) * RAD_TO_DEG);
    gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors 
}


void readCompass() {
  compass.readMag();
  if ((compass.m.x != mX) || (compass.m.y != mY) || (compass.m.z != mZ)) {
    mX = compass.m.x;
    mY = compass.m.y;
    mZ = compass.m.z;
 
    xVec = ((((float) (mX - xMin))/((float) (xMax - xMin))) * 2.0) - 1.0;
    yVec = ((((float) (mY - yMin))/((float) (yMax - yMin))) * 2.0) - 1.0;
    zVec = ((((float) (mZ - zMin))/((float) (zMax - zMin))) * 2.0) - 1.0;
    
    headX = xVec;
    headY = yVec;

    // tilt-compenstated magnetic heading
    float gaPitchRad = gaPitch * DEG_TO_RAD; 
    float gaRollRad = gaRoll * DEG_TO_RAD; 
    float cosPitch = cos(gaPitchRad);
    float sinPitch = sin(gaPitchRad);   
    float cosRoll = cos(gaRollRad);
    float sinRoll = sin(gaRollRad);
  
    headX = (xVec * cosPitch) + (yVec * sinRoll * sinPitch) + (zVec * cosRoll * sinPitch);
    headY = (yVec * cosRoll) - (zVec * sinRoll);
  
    float mh = atan2(-headY, headX) * RAD_TO_DEG;
    
    if ((mh > 90.0) && (magHeading < -90.0)) { // Crossed rotating in positive direction?
      magRotations -= 360.0;
    }
    else if ((mh < -90.0) && (magHeading > 90.0)) {
      magRotations += 360.0;
    }
    magHeading = mh;
    magCumHeading = magRotations + magHeading;
  }
}

/***********************************************************************.
 *  setHeading() Set gmHeading, tmHeading, tickHeading.
 ***********************************************************************/
#define GM_HEADING_TC 0.995
#define TM_HEADING_TC 0.995
void setHeading() {
  
  // Tick heading.
  tickCumHeading =  ((float) (tickHeadingOffset + tickPositionLeft - tickPositionRight)) / TICKS_PER_DEGREE_YAW;
  float c = (tickCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((tickCumHeading + c) / 360.0);
  tickHeading = tickCumHeading - (((float) rotations) * 360.0);
  
  // gmHeading. Complementary filter gyro and mag headings.
  float q = gmCumHeading + gyroCumHeading - oldGyroCumHeading;
  oldGyroCumHeading = gyroCumHeading;
  gmCumHeading = (q * GM_HEADING_TC) + (magCumHeading * (1.0 - GM_HEADING_TC));
  c = (gmCumHeading > 0.0) ? 180.0 : -180.0;
  rotations = (int) ((gmCumHeading + c) / 360.0);
  gmHeading = gmCumHeading - (((float) rotations) * 360.0);
  
  // tmHeading.  Complementary filter tick and mag headings.
  q = tmCumHeading + tickCumHeading - oldTickCumHeading;
  oldTickCumHeading = tickCumHeading;
  tmCumHeading = (q * TM_HEADING_TC) + (magCumHeading * (1.0 - TM_HEADING_TC));
  c = (tmCumHeading > 0.0) ? 180.0 : -180.0;
  rotations = (int) ((tmCumHeading + c) / 360.0);
  tmHeading = tmCumHeading - (((float) rotations) * 360.0);
}


/**************************************************************************.
 * zeroTickHeading() Set tickHeading and gyroHeading to magHeading 
 *                   when wheels touch ground or at startup.
 **************************************************************************/
void zeroHeadings() {
  gmCumHeading = tmCumHeading = gyroCumHeading = tickCumHeading = magCumHeading = magHeading;
  oldGyroCumHeading = oldTickCumHeading = magHeading;
  tickHeading = gyroHeading = magHeading;
  magRotations = 0.0;
  tickHeadingOffset = (int) (magCumHeading * TICKS_PER_DEGREE_YAW);
  tickPosition = tickPositionRight = tickPositionLeft = 0;
}

