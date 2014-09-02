//________ Pololu MinIMU-9 v2 --------
//#include <Wire.h>
//#include <L3G.h>
//#include <LSM303.h>
//
//L3G gyro;
//LSM303 compass;

// These are min/max for complete sphere
const int rollMin = -692;
const int rollMax = 430;
const int pitchMin = -782;
const int pitchMax = 439;
const int yawMin = -628;
const int yawMax = 403;

///******************************* from old code *****************************************************
// *  // TODO revisit these parameters
// *  compass.init(LSM303DLHC_DEVICE, 0);
// *  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
// *  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
// *  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
// *  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
// *  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth
// *//  gyro.writeReg(L3G_CTRL_REG2, 0x00); // 250 dps full scale
// *//  gyro.writeReg(L3G_CTRL_REG5, 0x10); // high-pass enable
// *//  gyro.writeReg(L3G_CTRL_REG2, 0x03); // high-pass frequency
// *//  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
// **********************************************************************************************

//
//
////---------- Uncomment everything below this line for the Sparkfun imu9150 --------

void angleInit() {
  Wire.begin();
  imu9150.initialize();
  imu9150.setRate(79);
}

void angleInitTp7() {
  Wire.begin();
  imu9150.initialize();
//  imu9150.setRate(39);
  imu9150.setRate(79);
  imu9150.setIntDataReadyEnabled(true);
}

//void angleInit() {
//  Wire.begin();
//  delay(100);
//  compass.init(LSM303DLHC_DEVICE,LSM303_SA0_A_HIGH);
//  compass.enableDefault();
//  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
//  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
//  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
//  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth
//}


float old1DeltaOverBase = 0.0;
float old2DeltaOverBase = 0.0;

long oldTp5TickDistance = 0L;

/*********************************************************
 * getTp5Angle()
 *********************************************************/
float getTp5Angle() {
static int mAverageFpsLeftOld = 0;
  // Compute the tickHeading.
  long td = (tickDistanceLeft - tickDistanceRight) % TICKS_PER_360_YAW;
  if (td < 0) td += TICKS_PER_360_YAW;
  tickHeading = magCorrection + (((float) td) / TICKS_PER_YAW_DEGREE);

  //  getIMU(&aPitch, &aRoll, &aYaw, &gPitch, &gRoll, &gYaw, &mPitch, &mRoll, &mYaw);
  imu9150.getMotion6(&aPitch, &aRoll, &aYaw, &gPitch, &gRoll, &gYaw);

//  // get drift on pitch angle
//  static int loopc;
//  static int loopsum;
//  loopsum += gPitch;
//  loopc++;
//  if (loopc > 99) {
//    Serial.println(loopsum / 100);
//    loopc = 0;
//    loopsum = 0;
//  }

  // Compute angle around the x axis
  gyroPitchRaw = gPitch + 333;  // add in constant error
  gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec

  gyroPitchAngleDelta = (gyroPitchRate * actualLoopTime) / 1000000; // degrees changed during period
  gyroPitchAngle = gyroPitchAngle + gyroPitchAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroPitchWeightedAngle = gyroPitchAngleDelta + gaPitchAngle;  // used in weighting final angle
  accelPitchAngle = ((atan2(-aRoll, aYaw)) * -RAD_TO_DEG) + (*currentValSet).z + (((float) zVal) / 1000.0); // angle from accelerometer
  gaPitchAngle = (gyroPitchWeightedAngle * GYRO_WEIGHT) + (accelPitchAngle * (1 - GYRO_WEIGHT)); // Weigh factors
  
//  // accel modified
//  mAccelLeft = mAverageFpsLeft - mAverageFpsLeftOld;
// 
//  int aRoll2 = aRoll + (mAccelLeft * 40);
////  int aRoll2 = aRoll + (accelLeft * 60000);
//  float gyroPitchWeightedAngle2 = gyroPitchAngleDelta + gaPitchAngle2;  // used in weighting final angle
//  accelPitchAngle2 = ((atan2(-aRoll2, aYaw)) * -RAD_TO_DEG) + (*currentValSet).z + (((float) zVal) / 1000.0); // angle from accelerometer
//  gaPitchAngle2 = (gyroPitchWeightedAngle2 * GYRO_WEIGHT) + (accelPitchAngle2 * (1 - GYRO_WEIGHT)); // Weigh factors
//  mAverageFpsLeftOld = mAverageFpsLeft;

  // Add the tick information to compensate for gyro information being 40ms late.
  //  tickDistance = tickDistanceLeft + tickDistanceRight;
  //  tp5TickRate = oldTp5TickDistance - tickDistance;
  //  oldTp5TickDistance = tickDistance;
  //  tp5IntTickRate = (((float)(tp5TickRate - tp5IntTickRate)) * .2) + tp5IntTickRate;
  //  deltaOverBase = (tp5TickRate - tp5IntTickRate) * 0.05;
  //  deltaSum += deltaOverBase;
  //  deltaSum -= old2DeltaOverBase;
  //  gaPitchTickAngle = gaPitchAngle + deltaSum; // Causing problems over cracks in surface.
  ////  gaPitchTickAngle = gaPitchAngle;
  //  old2DeltaOverBase = old1DeltaOverBase;
  //  old1DeltaOverBase = deltaOverBase;

  // compute the Y plane to check for falling sideways
  gyroRollRaw = gRoll;
  gyroRollRate = gyroRollRaw * GYRO_SENS;
  float gyroRollAngleDelta = (gyroRollRate * actualLoopTime) / 1000000;
  gyroRollAngle = gyroRollAngle + gyroRollAngleDelta; // not used
  float gyroRollWeightedAngle = gyroRollAngleDelta + gaRollAngle;
  accelRollAngle = atan2(aPitch, aYaw) * RAD_TO_DEG;
  gaRollAngle = (gyroRollWeightedAngle * GYRO_WEIGHT) + (accelRollAngle * (1 - GYRO_WEIGHT));

  // compute Z plane to measure turns
  gyroYawRaw = -gYaw;
  gyroYawRate = (gyroYawRaw - driftYaw) * GYRO_SENS;
  float gyroYawAngleDelta = (gyroYawRate * actualLoopTime) / 1000000;
  gyroYawAngle = gyroYawAngle + gyroYawAngleDelta;

  getCompass();
}


/*********************************************************
 * getTp7Angle()
 *********************************************************/
//float getTp7Angle() {
//  imu9150.getMotion6(&aPitch, &aRoll, &aYaw, &gPitch, &gRoll, &gYaw);
//
//  // Compute angle around the x axis
//  gyroPitchRaw = gPitch;  //
//  gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec
//  gyroPitchDelta = (gyroPitchRate * ((float) actualLoopTime))/1000000.0f; // degrees changed during period
//  gyroPitch = gyroPitch + gyroPitchDelta;   // Not used.  Only for debuggin purposes
//  float weightedGyroPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
//  accelPitch = ((atan2(-aRoll, aYaw))*-RAD_TO_DEG) + (*currentValSet).z;  // angle from accelerometer
//  gaPitch = (weightedGyroPitch * GYRO_WEIGHT) + (accelPitch * (1 - GYRO_WEIGHT)); // Weigh factors
//}



/*********************************************************
 *
 * getCompass()
 *
 *     Computes the heading. Does compensation for tilt.
 *     TODO: Don't need to do this every time
 *     since a call to the compass will return a new value
 *     infrequently.
 *
 *********************************************************/
void getCompass() {
  // Scale for hard-iron effects
  mPitchVec = ((((float) (mPitch - pitchMin)) / ((float) (pitchMax - pitchMin))) * 2.0) - 1.0;
  mRollVec = ((((float) (mRoll - rollMin)) / ((float) (rollMax - rollMin))) * 2.0) - 1.0;
  mYawVec = ((((float) (mYaw - yawMin)) / ((float) (yawMax - yawMin))) * 2.0) - 1.0;

  // TODO redo this to increase efficiency.
  // TODO add roll calculations for uneven terrain
  //  float cosRoll = cos(DEG_TO_RAD * gaRollAngle);
  //  float sinRoll = 1.0 - (cosRoll * cosRoll);
  //  float cosPitch = cos(DEG_TO_RAD * gaPitchAngle);
  //  float sinPitch = 1.0 - (cosPitch * cosPitch);

  headX = (mRollVec * 1.0) + (mPitchVec * sin(DEG_TO_RAD * gaPitchAngle) * 0.0) + (mYawVec * cos(DEG_TO_RAD * gaPitchAngle) * 0.0);
  headY = (mPitchVec * cos(DEG_TO_RAD * gaPitchAngle)) - (mYawVec * sin(DEG_TO_RAD * gaPitchAngle));

  // magnetic heading
  magHeading = atan2(headX, -headY) * RAD_TO_DEG;
  if (magHeading < 0) magHeading += 360.0;
}

/* tilt-compensated e-Compass code */


/*********************************************************
 *
 * ieCompass()

 *     from http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
 *
 *********************************************************/
void iecompass(int iBpx, int iBpy, int iBpz, int iGpx, int iGpy, int iGpz) {
  /* stack variables */
  /* iBpx, iBpy, iBpz: the three components of the magnetometer sensor */
  /* iGpx, iGpy, iGpz: the three components of the accelerometer sensor */
  /* local variables */
  float iSin, iCos; /* sine and cosine */

  /* calculate current roll angle Phi */
  iPhi = atan2(iGpy, iGpz);

  /* calculate sin and cosine of roll angle Phi */
  iSin = sin(iGpy / iGpz);                         /* Eq 13: sin = opposite / hypotenuse */
  iCos = cos(iGpz / iGpy);                         /* Eq 13: cos = adjacent / hypotenuse */

  /* de-rotate by roll angle Phi */
  iBfy = (iBpy * iCos) - (iBpz * iSin);     /* Eq 19 y component */
  iBpz = (iBpy * iSin) + (iBpz * iCos);     /* Bpy*sin(Phi)+Bpz*cos(Phi)*/
  iGpz = (iGpy * iSin) + (iGpz * iCos);     /* Eq 15 denominator */

  /* calculate current pitch angle Theta */
  iThe = atan2(-iGpx, iGpz);                         /* Eq 15 */

  /* calculate sin and cosine of pitch angle Theta */
  iSin = sin(iGpx / iGpz);                          /* Eq 15: sin = opposite / hypotenuse */
  iCos = cos(iGpz / iGpx);                          /* Eq 15: cos = adjacent / hypotenuse */

  /* de-rotate by pitch angle Theta */
  iBfx = (iBpx * iCos + iBpz * iSin);                /* Eq 19: x component */
  iBfz = (-iBpx * iSin + iBpz * iCos);               /* Eq 19: z component */

  /* calculate current yaw = e-compass angle Psi */
  iPsi = atan2(-iBfy, iBfx);                        /* Eq 22 */
}

/*********************************************************
 *
 * razorCompass()

 *     from https://code.google.com/p/sf9domahrs/
 *
 *********************************************************/
void razorCompass() {
  float mag_x, mag_y, mag_z;
  float pitch, roll;
  //We have a full working version of the Razor 9DOF IMU AHRS :-).
  //It´s uploaded in the Google Code repository at right.
  //
  //I have included the code to read the I2C compass (HMC5843) and map the
  //sensor axis and sensor signs (it was a little tricky...).
  //
  //I have implemented the "fusion" between the 3 axis magnetometer
  //info with our actual 6 DOF IMU code. This was my approach:
  //
  //--Read the 3 axis magnetometer data
  //
  //--Calculate the "tilt compensated" x and y magnetic
  //component (standard formulation):
  //
  float  CMx = (mag_x * cos(pitch)) + (mag_y * sin(roll) * sin(pitch)) + (mag_z * cos(roll) * sin(pitch));
  float   CMy = (mag_y * cos(roll)) - mag_z * sin(roll);
  //--Calculate the magnetic heading with this compensated components:
  //
  float MAG_Heading = atan(CMy / CMx);
  //Now MAG_Heading is "like" our GPS_ground_course, so I use the
  //same yaw drift correction formulas that we already use with ArduIMU+ but with magnetic heading instead of GPS_ground_course.
}

/*********************************************************
 *
 * apCompass()
 *
 *     from http://diydrones.com/forum/topics/heading-from-3d-magnetometer
 *
 *********************************************************/
float mag_x, mag_y, mag_z;
float _declination = 0.0;
float heading_x, heading_y;
void apCompass(float roll, float pitch) {
  float headX;
  float headY;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(roll);  // Optimizacion, se puede sacar esto de la matriz DCM?
  sin_roll = 1  - (cos_roll * cos_roll);
  cos_pitch = cos(pitch);
  sin_pitch = 1  - (cos_pitch * cos_pitch);

  // Tilt compensated magnetic field X component:
  headX = (mag_x * cos_pitch) + (mag_y * sin_roll * sin_pitch) + (mag_z * cos_roll * sin_pitch);
  // Tilt compensated magnetic field Y component:
  headY = mag_y * cos_roll - mag_z * sin_roll;
  // magnetic heading
  magHeading = atan2(-headY, headX);

  // Declination correction (if supplied)
  if ( abs(_declination) > 0.0 )
  {
    magHeading = magHeading + _declination;
    if (magHeading > M_PI)    // Angle normalization (-180 deg, 180 deg)
      magHeading -= (2.0 * M_PI);
    else if (magHeading < -M_PI)
      magHeading += (2.0 * M_PI);
  }

  // Optimization for external DCM use. Calculate normalized components
  heading_x = cos(magHeading);
  heading_y = sin(magHeading);
}




