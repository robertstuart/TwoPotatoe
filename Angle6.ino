
void angleInitTp6() {
  Wire.begin();
  imu9150.initialize();
  imu9150.setRate(79);
  imu9150.setIntDataReadyEnabled(true);
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

  imu9150.getMotion6(&aPitch, &aRoll, &aYaw, &gPitch, &gRoll, &gYaw);

  // Compute angle around the x axis
  gyroPitchRaw = gPitch + 333;  // add in constant error
  gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec

  gyroPitchAngleDelta = (gyroPitchRate * actualLoopTime) / 1000000; // degrees changed during period
  gyroPitchAngle = gyroPitchAngle + gyroPitchAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroPitchWeightedAngle = gyroPitchAngleDelta + gaPitchAngle;  // used in weighting final angle
  accelPitchAngle = ((atan2(-aRoll, aYaw)) * -RAD_TO_DEG) + (*currentValSet).z + (((float) zVal) / 1000.0); // angle from accelerometer
  gaPitchAngle = (gyroPitchWeightedAngle * GYRO_WEIGHT) + (accelPitchAngle * (1 - GYRO_WEIGHT)); // Weigh factors
  
  // accel modified
  mAccelLeft = mAverageFpsLeft - mAverageFpsLeftOld;
 
//  int aRoll2 = aRoll + (mAccelLeft * 40);
  int aRoll2 = aRoll + (accelLeft * 60000);
  float gyroPitchWeightedAngle2 = gyroPitchAngleDelta + gaPitchAngle2;  // used in weighting final angle
  accelPitchAngle2 = ((atan2(-aRoll2, aYaw)) * -RAD_TO_DEG) + (*currentValSet).z + (((float) zVal) / 1000.0); // angle from accelerometer
  gaPitchAngle2 = (gyroPitchWeightedAngle2 * GYRO_WEIGHT) + (accelPitchAngle2 * (1 - GYRO_WEIGHT)); // Weigh factors
  mAverageFpsLeftOld = mAverageFpsLeft;

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

//  getCompass();
}


