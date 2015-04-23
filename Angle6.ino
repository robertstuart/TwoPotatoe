/***********************************************************************.
 *  Angle6 
 ***********************************************************************/

void angleInit6() {
  Wire.begin();
  compass.init();
  compass.enableDefault(); // Mag: [DR=6.25 Hz, 4 gauss, contin] Accel: [2g]
  compass.writeAccReg(LSM303::CTRL1, 0x67); // Accel DR=100 HZ, all axis
  gyro.init();
  gyro.writeReg(L3G::CTRL1, 0xBF); // power, all axes, DR=400 Hz, BW=110
  gyro.writeReg(L3G::CTRL4, 0x20); // 2000 dps full scale
}

void readGyro() {
    actualLoopTime = timeMicroseconds - oldTimeTrigger;
    oldTimeTrigger = timeMicroseconds;
    gyro.read();  // 860 microseconds
    gPitch = gyro.g.x;
    gRoll = gyro.g.y;
    gYaw = -gyro.g.z;
    gyroPitchRaw = gPitch - 210;  // add in constant error
    gyroPitchRate = ((float) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec
    gyroPitchDelta = (gyroPitchRate * 2500.0) / 1000000.0; // degrees changed during period
    gyroPitch = gyroPitch + gyroPitchDelta;   // Not used.  Only for debugging purposes
    gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
}

boolean readAccel() {
    compass.readAcc(); // 848 microseconds
    aRoll = -((int) (compass.a.x));
    aPitch = -((int) (compass.a.y));
    aPitchRoll = -(int) (compass.a.z);
    float k8 = 45.5;  // for new MinImu
    accelPitch =  (atan2(aPitch, aPitchRoll) * RAD_TO_DEG) + (*currentValSet).z;
    accelPitchComp =  ((atan2((aPitch + (k8 * 1000.0 * tp6LpfCosAccel)), aPitchRoll)) * RAD_TO_DEG) + (*currentValSet).z;
//    gaPitch = (gaPitch * GYRO_WEIGHT) + (accelPitch * (1 - GYRO_WEIGHT)); // Weigh factors
    gaPitch = (gaPitch * GYRO_WEIGHT) + (accelPitchComp * (1 - GYRO_WEIGHT)); // Weigh factors
}

boolean readCompass() {
  return false;
}
