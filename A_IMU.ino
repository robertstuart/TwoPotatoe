/************************************************************************
 * ----- IMU -----
 * This routine is for testing the IMU.
 * 
 * 
 ************************************************************************/
#define IMU_LOOP_TIME 100000L;        // Microsecons/loop, 10/sec

void aImuInit() {
  loopTime = IMU_LOOP_TIME;
  accelgyro.initialize();
}

void aImuStart() {
}

void aImu() {
   int16_t ax, ay, az;
  int16_t gx, gy, gz;
 unsigned long t = micros();
//  gyro.readX();   
//  compass.readAcc();  
//  compass.readAccYZ();
//  compass.readAccY();
//    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx = accelgyro.getRotationX();

//  getTpAngle();
//  debugLong("Gyro time: ", micros() - t);
//  debugInt("Y: ", ay);

//    debugInt("Gyro.x: ", gx);
//  debugInt("Z: ", compass.a.z);

}

