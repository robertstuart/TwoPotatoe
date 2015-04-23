//________ Pololu MinIMU-9 v3 --------
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

// These are min/max for complete sphere
const int rollMin = -692;
const int rollMax = 430;
const int pitchMin = -782;
const int pitchMax = 439;
const int yawMin = -628;
const int yawMax = 403;


float mPitchVec, mRollVec, mYawVec;

void angleInit() {
//  Wire.begin();
//  delay(100);
//  compass.init();
//  compass.writeAccReg(LSM303::CTRL1, 0x67); // 100 HZ, all axis enabled
//  compass.writeAccReg(LSM303::CTRL0, 0x40); // fifo enable
//  gyro.init();
//  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//  gyro.writeReg(L3G_CTRL_REG5, 0x40); // fifo enabled
}


/*********************************************************
 * getTp5Angle()
// *********************************************************/
//float getAngle() {
//static int mAverageFpsLeftOld = 0;
//  // Compute the tickHeading.
//  long td = (tickPositionLeft - tickPositionRight) % TICKS_PER_360_YAW;
//  if (td < 0) td += TICKS_PER_360_YAW;
//  tickHeading = magCorrection + (((float) td) / TICKS_PER_YAW_DEGREE);
//
//  // Compute pitch
//  gyroPitchRaw = gPitch + 210;  // add in constant error
//  gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec
//  gyroPitchDelta = (gyroPitchRate * actualLoopTime) / 1000000.0; // degrees changed during period
//  gyroPitch = gyroPitch + gyroPitchDelta;   // Not used.  Only for debugging purposes
//  float k8 = 45.5;  // for new MinImu
//  float gyroPitchWeightedAngle = gyroPitchDelta + gaPitch;  // used in weighting final angle
////  if (!isStateBit(TP_STATE_RUN_AIR)) accelPitchAngle =  ((atan2((aPitch + (k8 * 1000.0 * tp5LpfCosAccel)), aPitchRoll)) * RAD_TO_DEG) + (*currentValSet).z;
//  accelPitchAngle =  ((atan2((aPitch + (k8 * 1000.0 * tp5LpfCosAccel)), aPitchRoll)) * RAD_TO_DEG) + (*currentValSet).z;
////  accelPitchAngle =  ((atan2(aPitch, aPitchRoll)) * RAD_TO_DEG) + (*currentValSet).z;
//  gaPitch = (gyroPitchWeightedAngle * GYRO_WEIGHT) + (accelPitchAngle * (1 - GYRO_WEIGHT)); // Weigh factors
//
//  // compute the Y plane to check for falling sideways
//  gyroRollRaw = gRoll + 108;
//  gyroRollRate = gyroRollRaw * GYRO_SENS;
//  float gyroRollDelta = (gyroRollRate * actualLoopTime) / 1000000;
//  gyroRoll = gyroRoll + gyroRollDelta; // not used
//  float gyroWeightedRoll = gyroRollDelta + gaRoll;
//  accelRoll = atan2(aRoll, aPitchRoll) * RAD_TO_DEG;
//  gaRoll = (gyroWeightedRoll * GYRO_WEIGHT) + (accelRoll * (1 - GYRO_WEIGHT));
//
//  // compute Z plane to measure turns
//  gyroYawRaw = -gYaw - 85;
//  gyroYawRate = (gyroYawRaw - driftYaw) * GYRO_SENS;
//  float gyroYawDelta = (gyroYawRate * actualLoopTime) / 1000000;
//  gyroYawAngle = gyroYawAngle + gyroYawDelta;
//
//  getCompass();
//}
//
//

/*********************************************************
 *
 * getCompass()  Heading with tilt compensation.
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

  headX = (mRollVec * 1.0) + (mPitchVec * sin(DEG_TO_RAD * gaPitch) * 0.0) + (mYawVec * cos(DEG_TO_RAD * gaPitch) * 0.0);
  headY = (mPitchVec * cos(DEG_TO_RAD * gaPitch)) - (mYawVec * sin(DEG_TO_RAD * gaPitch));

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

///*********************************************************
// * readImu()
// * Read serial from the MinMImu
// *********************************************************/
//boolean readImu() {
//  static int bCycle = 0;
//  while (MINI_SER.available() > 0) {
//    byte b = MINI_SER.read();
//    switch (bCycle) {
//      case 0: // looking for 0x0A
//        if (b == 0x0A) {
//          bCycle++;
//        }
//        break;
//      case 1:
//        if (b == 0x0D) {
//          bCycle++;
//        } else {
//          bCycle = 0;
//        }
//        break;
//      case 2: // Accelerometer roll, msb ********** aRoll *******
////        aRoll = b * 256;
//        aRoll = ((unsigned int) b) << 8;
//        bCycle++;
//        break;
//      case 3: // Acceleromenter roll, lsb
//        aRoll += b;
//        bCycle++;
//        break;
//      case 4: // Accelerometer pitch, msb ********* aPitch ******
//        aPitch = ((unsigned int) b) << 8;
//        bCycle++;
//        break;
//      case 5: // Acceleromenter pitch, lsb
//        aPitch += b;
//        bCycle++;
//        break;
//      case 6: // Accelerometer pitch-roll, msb *** aPitchRoll ***
//        aPitchRoll = ((unsigned int) b) << 8;
//        bCycle++;
//        break;
//      case 7: // Acceleromenter pitch-roll, lsb
//        aPitchRoll += b;
//        bCycle++;
//        break;
//      case 8: // Gyro pitch, msb ****************** gPitch ******
//        gPitch = ((unsigned int) b) << 8;
//        bCycle++;
//        break;
//      case 9: // Gyro pich, lsb
//        gPitch += b;
//        bCycle++;
//        break;
//      case 10: // Gyro roll, msb ***************** gRoll ********
//        gRoll = ((unsigned int) b) << 8;
//        bCycle++;
//        break;
//      case 11: // Gyro roll, lsb
//        gRoll += b;
//        bCycle++;
//        break;
//      case 12: // Gyro yaw, msb ****************** gYaw *********
//        gYaw = ((unsigned int) b) << 8;
//        bCycle++;
//        break;
//      case 13: // Gyro yaw, lsb
//        gYaw += b;
//        bCycle = 0;
//        return true;
//        break;
//    }
//  }
//  return false;
//}


/*********************************************************
 * readImu()
 * Read I2C from the MinMImu
 *********************************************************/
boolean readImu() {  
  unsigned long t = micros();
  byte gyroStatus = gyro.readReg(0x27);
  if ((gyroStatus & 0x08) != 0) {
    gyro.read();
    gPitch = -gyro.g.x;
    gRoll = gyro.g.y;
    gYaw = -gyro.g.z;
    compass.readAcc();
    aRoll = -((int) (compass.a.x));
    aPitch = -((int) (compass.a.y));
    aPitchRoll = (int) (compass.a.z);
//    // check accelerometer after gyro read
//    byte accelStatus = compass.readReg(LSM303::STATUS_A);
//    if((accelStatus & 0x08) != 0) {
//    }
    return true;
  }
  return false;
}



