//________ Pololu MinIMU-9 v2 --------
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

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

void angleInit() {
  Wire.begin();
  delay(100);
  compass.init(LSM303DLHC_DEVICE, 0);
  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // normal power mode, all axes enabled, 100 Hz
  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyro.writeReg(L3G_CTRL_REG1, 0xFF); // high data rate & bandwidth
}

void getGyroAccel(int* ax, int* ay, int* az, int* gx, int* gy, int* gz) {
  flushSerial();
  compass.readAcc();
  flushSerial();
  gyro.read();
  flushSerial();
  *ax = compass.a.x;
  *ay = compass.a.y;
  *az = compass.a.z;
  *gx = gyro.g.x;
  *gy = gyro.g.y;
  *gz = gyro.g.z;
}


/*********************************************************
 *
 * getTpAngle()
 *
 *     Computes the angle from the accelerometer and the gyro.
 *     Sets: gyroRate
 *           tpAngle
 *
 *********************************************************/
float sumGyroRate;
float getTpAngle() {

  getGyroAccel(&ax, &ay, &az, &gx, &gy, &gz);

  // Compute angle around the x axis
  gyroXRaw = gx;  // 
  gyroXRate = gyroXRaw * GYRO_SENS;  // Rate in degreesChange/sec
  gyroXAngleDelta = (gyroXRate * actualLoopTime)/1000000; // degrees changed during period
  gyroXAngle = gyroXAngle + gyroXAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroXWeightedAngle = gyroXAngleDelta + gaXAngle;  // used in weighting final angle
  accelXAngle = ((atan2(-ay, az))*-RAD_TO_DEG) + (*currentValSet).z;  // angle from accelerometer
  gaXAngle = (gyroXWeightedAngle * GYRO_WEIGHT) + (accelXAngle * (1 - GYRO_WEIGHT)); // Weigh factors  

  // compute the Y plane to check for falling sideways
  gyroYRaw = gy;
  gyroYRate = gyroYRaw * GYRO_SENS;
  float gyroYAngleDelta = (gyroYRate * actualLoopTime)/1000000;
  gyroYAngle = gyroYAngle + gyroYAngleDelta; // not used
  float gyroYWeightedAngle = gyroYAngleDelta + gaYAngle;
  //  accelYAngle = atan2(compass.a.x, compass.a.z) * RAD_TO_DEG;
  accelYAngle = atan2(ax, az) * RAD_TO_DEG;
  gaYAngle = (gyroYWeightedAngle * GYRO_WEIGHT) + (accelYAngle * (1 - GYRO_WEIGHT));

  // compute Z plane to measure turns
  gyroZRaw = -gz;
  gyroZRate = (gyroZRaw - driftZ) * GYRO_SENS;
  float gyroZAngleDelta = (gyroZRate * actualLoopTime)/1000000;
  gyroZAngle = gyroZAngle + gyroZAngleDelta; 
}


float deltaStore[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int deltaStorePtr = 0;
long oldTp5TickDistance = 0L;
float tp5IntTickRate = 0.0;
float deltaSum = 0.0;

float getTp5Angle() {
  getGyroAccel(&ax, &ay, &az, &gx, &gy, &gz);

  // Compute angle around the x axis
  gyroXRaw = gx;  // 
  gyroXRate = gyroXRaw * GYRO_SENS;  // Rate in degreesChange/sec
  gyroXAngleDelta = (gyroXRate * actualLoopTime)/1000000; // degrees changed during period
  gyroXAngle = gyroXAngle + gyroXAngleDelta;   // Not used.  Only for debuggin purposes
  float gyroXWeightedAngle = gyroXAngleDelta + gaXAngle;  // used in weighting final angle
  accelXAngle = ((atan2(-ay, az))*-RAD_TO_DEG) + (*currentValSet).z;  // angle from accelerometer
  gaXAngle = (gyroXWeightedAngle * GYRO_WEIGHT) + (accelXAngle * (1 - GYRO_WEIGHT)); // Weigh factors  
  
  // Add the tick information to compensate for gyro information being 40ms late.
  long tp5TickDistance = tickDistanceLeft + tickDistanceRight;
  oldTp5TickDistance = tp5TickDistance;
  int tp5TickRate = oldTp5TickDistance - tp5TickDistance;
  float tp5IntTickRate = (((float)(tp5TickRate - tp5IntTickRate)) * .2) + tp5IntTickRate;
  float deltaOverBase = tp5TickRate - tp5IntTickRate;
  deltaSum += deltaOverBase;
  deltaSum -= deltaStore[deltaStorePtr];
  deltaStore[deltaStorePtr++] = deltaOverBase;
  deltaStorePtr = deltaStorePtr % 8;
  gaXTickAngle = gaXAngle + deltaSum;
  
  // compute the Y plane to check for falling sideways
  gyroYRaw = gy;
  gyroYRate = gyroYRaw * GYRO_SENS;
  float gyroYAngleDelta = (gyroYRate * actualLoopTime)/1000000;
  gyroYAngle = gyroYAngle + gyroYAngleDelta; // not used
  float gyroYWeightedAngle = gyroYAngleDelta + gaYAngle;
  accelYAngle = atan2(ax, az) * RAD_TO_DEG;
  gaYAngle = (gyroYWeightedAngle * GYRO_WEIGHT) + (accelYAngle * (1 - GYRO_WEIGHT));

  // compute Z plane to measure turns
  gyroZRaw = -gz;
  gyroZRate = (gyroZRaw - driftZ) * GYRO_SENS;
  float gyroZAngleDelta = (gyroZRate * actualLoopTime)/1000000;
  gyroZAngle = gyroZAngle + gyroZAngleDelta; 
}


//
//
////---------- Uncomment everything below this line for the Sparkfun imu9150 --------
//#include "Wire.h"
//#include "I2Cdev.h"
//#include "MPU6050.h"
//
//void angleInit() {
//  Wire.init();
//  imu9150.initialize();
//  imu9150.setRate(79);
//}
//
//void getGyroAccel(int* ax, int* ay, int* az, int* gx, int* gy, int* gz) {
//    imu9150.getMotion6(ax, ay, az, gx, gy, gz);
//}

