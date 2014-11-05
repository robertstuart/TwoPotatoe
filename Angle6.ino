
void angleInitTp6() {
  //  Wire.begin();
  //  imu9150.initialize();
  //  imu9150.setRate(79);
  //  imu9150.setIntDataReadyEnabled(true);
}

boolean readImu() {
  static int bCycle = 0;
  while (MINI_SER.available() > 0) {
    byte b = MINI_SER.read();
    switch (bCycle) {
      case 0: // looking for 0x0A
        if (b == 0x0A) {
          bCycle++;
        }
        break;
      case 1:
        if (b == 0x0D) {
          bCycle++;
        } else {
          bCycle = 0;
        }
        break;
      case 2: // Accelerometer roll, msb ********** aRoll *******
        aRoll = b * 256;
        bCycle++;
        break;
      case 3: // Acceleromenter roll, lsb
        aRoll += b;
        bCycle++;
        break;
      case 4: // Accelerometer pitch, msb ********* aPitch ******
        aPitch = b * 256;
        bCycle++;
        break;
      case 5: // Acceleromenter pitch, lsb
        aPitch += b;
        bCycle++;
        break;
      case 6: // Accelerometer pitch-roll, msb *** aPitchRoll ***
        aPitchRoll = b * 256;
        bCycle++;
        break;
      case 7: // Acceleromenter pitch-roll, lsb
        aPitchRoll += b;
        bCycle++;
        break;
      case 8: // Gyro pitch, msb ****************** gPitch ******
        gPitch = b * 256;
        bCycle++;
        break;
      case 9: // Gyro pich, lsb
        gPitch += b;
        bCycle++;
        break;
      case 10: // Gyro roll, msb ***************** gRoll ********
        gRoll = b * 256;
        bCycle++;
        break;
      case 11: // Gyro roll, lsb
        gRoll += b;
        bCycle++;
        break;
      case 12: // Gyro yaw, msb ****************** gYaw *********
        gYaw = b * 256;
        bCycle++;
        break;
      case 13: // Gyro yaw, lsb
        gYaw += b;
        bCycle = 0;
        return true;
        break;
    }
  }
  return false;
}


long oldTp6TickDistance = 0L;

/*********************************************************
 * getTp6Angle()
 *********************************************************/
float getTp6Angle() {
  static int mAverageFpsLeftOld = 0;
  // Compute the tickHeading.
  long td = (tickDistanceLeft - tickDistanceRight) % TICKS_PER_360_YAW;
  if (td < 0) td += TICKS_PER_360_YAW;
  tickHeading = magCorrection + (((float) td) / TICKS_PER_YAW_DEGREE);

  // Compute angle around the x axis
  gyroPitchRaw = gPitch + 333;  // add in constant error
  gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec

  gyroPitchAngleDelta = (gyroPitchRate * actualLoopTime) / 1000000; // degrees changed during period
  gyroPitchAngle = gyroPitchAngle + gyroPitchAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroPitchWeightedAngle = gyroPitchAngleDelta + gaPitchAngle;  // used in weighting final angle
  accelPitchAngle = ((atan2(-aPitch, aPitchRoll)) * -RAD_TO_DEG) + (*currentValSet).z + (((float) zVal) / 1000.0); // angle from accelerometer
  gaPitchAngle = (gyroPitchWeightedAngle * GYRO_WEIGHT) + (accelPitchAngle * (1 - GYRO_WEIGHT)); // Weigh factors

  // accel modified
  mAccelLeft = mAverageFpsLeft - mAverageFpsLeftOld;

  //  int aRoll2 = aRoll + (mAccelLeft * 40);
  int aRoll2 = aRoll + (accelLeft * 60000);
  float gyroPitchWeightedAngle2 = gyroPitchAngleDelta + gaPitchAngle2;  // used in weighting final angle
  accelPitchAngle2 = ((atan2(-aRoll, aPitchRoll)) * -RAD_TO_DEG) + (*currentValSet).z + (((float) zVal) / 1000.0); // angle from accelerometer
  gaPitchAngle2 = (gyroPitchWeightedAngle2 * GYRO_WEIGHT) + (accelPitchAngle2 * (1 - GYRO_WEIGHT)); // Weigh factors
  mAverageFpsLeftOld = mAverageFpsLeft;

  // compute the Y plane to check for falling sideways
  gyroRollRaw = -gRoll - 80;
  gyroRollRate = gyroRollRaw * GYRO_SENS;
  float gyroRollAngleDelta = (gyroRollRate * actualLoopTime) / 1000000;
  gyroRollAngle = gyroRollAngle + gyroRollAngleDelta; // not used
  float gyroRollWeightedAngle = gyroRollAngleDelta + gaRollAngle;
  accelRollAngle = atan2(aRoll, aPitchRoll) * RAD_TO_DEG;
  gaRollAngle = (gyroRollWeightedAngle * GYRO_WEIGHT) + (accelRollAngle * (1 - GYRO_WEIGHT));

  // compute Z plane to measure turns
  gyroYawRaw = -gYaw + 80;
  gyroYawRate = (gyroYawRaw - driftYaw) * GYRO_SENS;
  float gyroYawAngleDelta = (gyroYawRate * actualLoopTime) / 1000000;
  gyroYawAngle = gyroYawAngle + gyroYawAngleDelta;

  //  getCompass();
}


