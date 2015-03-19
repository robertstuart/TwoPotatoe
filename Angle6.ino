/***********************************************************************.
 *  Angle6 
 ***********************************************************************/

void angleInit6() {
  Wire.begin();
  delay(100);
  compass.init();
  compass.writeAccReg(LSM303::CTRL1, 0x67); // 100 HZ, all axis enabled
//  compass.writeAccReg(LSM303::CTRL0, 0x40); // fifo enable
  gyro.init();
  // the following parameter can be 8F,9F,AF, or BF
  gyro.writeReg(L3G_CTRL_REG1, 0x8F); // normal power mode, all axes enabled, 400 Hz
//  gyro.writeReg(L3G_CTRL_REG5, 0x40); // fifo enabled
}

void readGyro() {
//  byte gyroStatus = gyro.readReg(0x27);  // 603 microseconds
//  if ((gyroStatus & 0x08) != 0) {
    actualLoopTime = timeMicroseconds - oldTimeTrigger;
    oldTimeTrigger = timeMicroseconds;
    gyro.read();  // 860 microseconds
    gPitch = -gyro.g.x;
    gRoll = gyro.g.y;
    gYaw = -gyro.g.z;
    gyroPitchRaw = gPitch + 210;  // add in constant error
    gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec
    gyroPitchDelta = (gyroPitchRate * actualLoopTime) / 1000000.0; // degrees changed during period
    gyroPitch = gyroPitch + gyroPitchDelta;   // Not used.  Only for debugging purposes
    gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
//    return true;
//  }
//  else return false;
}

boolean readAccel() {
//  int accelStatus = compass.readReg(LSM303::STATUS_A);  // 603 microseconds
//  if ((accelStatus & 0x08) != 0) {
    compass.readAcc(); // 848 microseconds
    aRoll = -((int) (compass.a.x));
    aPitch = -((int) (compass.a.y));
    aPitchRoll = (int) (compass.a.z);
    float k8 = 45.5;  // for new MinImu
    accelPitchAngle =  ((atan2((aPitch + (k8 * 1000.0 * tp6LpfCosAccel)), aPitchRoll)) * RAD_TO_DEG) + (*currentValSet).z;
    gaPitch = (gaPitch * GYRO_WEIGHT) + (accelPitchAngle * (1 - GYRO_WEIGHT)); // Weigh factors
//    return true;
//  }
//  else return false;
}

boolean readCompass() {
  return false;
}
