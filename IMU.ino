/*****************************************************************************-
 *                                   IMU.ino
 *****************************************************************************/
//#define MPU9250_ADDRESS   MPU9250_ADDRESS_AD0 // 0x68
#define sampleFreq  200.0f      // sample frequency in Hz

//LSM6 lsm6;
//MPU9250 imu(MPU9250_ADDRESS, Wire, 400000);
MPU9250_DMP imu;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
int zSum = 0;
int zCount = 0;
int yawTempComp = 0;


// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
//float GyroMeasError = PI * (5.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float zeta = 0;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.05f;                              // integration interval for both filter schemes
float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

/*****************************************************************************-
    imuInit()
 *****************************************************************************/
void imuInit() {
  pinMode(IMU_INT_PIN, INPUT);
  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  imu.setSampleRate(200); // Set accel/gyro sample rate to 4Hz
  imu.setAccelFSR(8);
  imu.setGyroFSR(2000);
  imu.setLPF(98);
  imu.enableInterrupt();
  imu.setIntLevel(INT_ACTIVE_HIGH);
  imu.setIntLatched(INT_LATCHED);

  // Clear out
  delay(10);
  imu.update(UPDATE_ACCEL | UPDATE_GYRO);
  delay(10);
  imu.update(UPDATE_ACCEL | UPDATE_GYRO);
  gaPitch = cfPitch = aPitch;
}


#define TG_PITCH_TC 0.90D



/*****************************************************************************-
    isNewImu()   Returns true if the IMU has new data.
                Reads the IMU and sets the new values.
 *****************************************************************************/
boolean isNewImu() {
  static int oldT = 0;

  if (digitalRead(IMU_INT_PIN) == HIGH) {
    imuZeroDrift(); //One cycle late?

    imu.update(UPDATE_ACCEL | UPDATE_GYRO);

    accelX = ((float) imu.ax) * ACCEL_SENSE;
    accelY = ((float) imu.ay) * ACCEL_SENSE;
    accelZ = ((float) imu.az) * ACCEL_SENSE;

    gyroPitchRaw = (float) imu.gx;
    float gyroPitchComp = gyroPitchRaw - timeDriftPitch;  // drift compenstation
    float gyroX  = gyroPitchComp * GYRO_SENS;             // degrees/sec
    float gyroXrad = gyroX * DEG_TO_RAD;                  // radians/sec

    gyroRollRaw = (float) imu.gy;
    float gyroRollComp = gyroRollRaw - timeDriftRoll;     // drift compenstation
    float gyroY  = gyroRollComp * GYRO_SENS;              // degrees/sec
    float gyroYrad = gyroY * DEG_TO_RAD;                  // radians/sec

    gyroYawRaw = (float) imu.gz;
    float gyroYawComp = gyroYawRaw - timeDriftYaw;         // drift compenstation
    float gyroZ  = gyroYawComp * GYRO_SENS;                // degrees/sec
    float gyroZrad = gyroZ * DEG_TO_RAD;                   // radians/sec

    compFilter(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
    //    MadgwickQuaternionUpdate(accelX, accelY, accelZ, gyroXrad, gyroYrad, gyroZrad);
    MahonyAHRSupdateIMU(gyroXrad, gyroYrad, gyroZrad, accelX, accelY, accelZ);

    maPitch  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    maRoll = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    maYaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    maPitch *= RAD_TO_DEG;
    maRoll  *= RAD_TO_DEG;
    maYaw   *= RAD_TO_DEG;

    accelSet(accelX, accelY, accelZ);  // Compute accelerometer horizontal velocity & vertical g.

//    sprintf(message, "%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f", maPitch, maRoll, maYaw, gaPitch, gaRoll, gYaw);
//    Serial.println(message);

    return true;
  } else {
    return false; // no IMU read
  }
}



/*****************************************************************************-
 *  compFilter()  Complementary filter to get angles
 *****************************************************************************/
void compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ) {

  gyroPitchDelta = -gyroX / sampleFreq; // degrees changed during period
  gPitch += gyroPitchDelta;   // Debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle

  float gyroRollDelta = gyroY / sampleFreq;
  gRoll += gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  float gyroYawDelta = -gyroZ / sampleFreq; // degrees changed during period
  gYaw += gyroYawDelta;
  gHeading = rangeAngle(gYaw);  // This is the heading used by all navigation routines.

  aPitch = ((atan2(-accelY, accelZ)) * RAD_TO_DEG);
  gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
 
  // Roll
  aRoll =  (atan2(accelY, accelZ) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}



/*****************************************************************************-
 *  accelSet()  compute the horizontal velocity from the accelerometers.
 *             Cancel drift by adjusting from wheel speed when appropriate.
 *****************************************************************************/
void accelSet(float accelX, float accelY, float accelZ) {
  const float HV_FACTOR = -0.15;

//  static int lCount = 0;
//  if ((lCount++ % 1000) == 0) aFps = wFps;
  
  float theta = maPitch * DEG_TO_RAD;
  float cosMa = cos(theta);
  float sinMa = sin(theta);
  float ha = (cosMa * accelY) + (sinMa * accelZ);
  vAccel = (cosMa * accelZ) - (sinMa * accelY);
  aFps = aFps + (ha * HV_FACTOR);

//sprintf(message, "%6.3f,%6.3f,%7.3f,%7.3f", accelY, accelZ, theta, wFps);
//Serial.println(message);
//sendUpMsg(TOUP_LOG, message);
}



const int DRIFT_SIZE = 100;  // 1/2 second of data
float pitchArray[DRIFT_SIZE];
float rollArray[DRIFT_SIZE];
float yawArray[DRIFT_SIZE];
/*****************************************************************************-
 * imuZeroDrift()  Called 200/sec.  Average gyroDrift for x, y & z
 *              for 1/2 second periods.  Also, take average of accelerometer
 *              readings and set -----------
 *****************************************************************************/
void imuZeroDrift() {
  static int gPtr = 0;
  static float aXSum, aZSum;
  float pitch, roll, yaw;

  if (gPtr == 0) {
    aXSum = 0.0;
    aZSum = 0.0;
  }
  pitchArray[gPtr] = gyroPitchRaw;
  rollArray[gPtr] = gyroRollRaw;
  yawArray[gPtr] = gyroYawRaw;
  aXSum += accelX;
  aZSum += accelZ;
  gPtr++;

  if (gPtr >= DRIFT_SIZE) {
    gPtr = 0;
    float pitchSum = 0;
    float rollSum = 0;
    float yawSum = 0;
    float pitchMax = pitchArray[0];
    float pitchMin = pitchMax;
    float rollMax = rollArray[0];
    float rollMin = rollMax;
    float yawMax = yawArray[0];
    float yawMin = yawMax;
    for (int i = 0; i < DRIFT_SIZE; i++) {
      pitch = pitchArray[i];
      if (pitch > pitchMax)  pitchMax = pitch;
      if (pitch < pitchMin)  pitchMin = pitch;
      pitchSum += pitch;
      roll = rollArray[i];
      if (roll > rollMax)  rollMax = roll;
      if (roll < rollMin)  rollMin = roll;
      rollSum += roll;
      yaw = yawArray[i];
      if (yaw > yawMax)  yawMax = yaw;
      if (yaw < yawMin)  yawMin = yaw;
      yawSum += yaw;
    }
    float pitchAve = ((float) pitchSum) / ((float) DRIFT_SIZE);
    float rollAve = ((float) rollSum) / ((float) DRIFT_SIZE);
    float yawAve = ((float) yawSum) / ((float) DRIFT_SIZE);

    // If we have a stable 0.5 second period, average the most recent 20 periods & adjust drift.
    if (((pitchMax - pitchMin) < 20) && ((rollMax - rollMin) < 20) && ((yawMax - yawMin) < 20)) {
      setDrift(pitchAve, rollAve, yawAve);
      aAvgPitch = ((atan2(aXSum, aZSum)) * RAD_TO_DEG) + valZ;
      //Serial.println(aAvgPitch);
    }
    //        sprintf(message, "pitchMin: %4.1f     pitchMax: %4.1f     pitchAve: %5.1f   ", pitchMin, pitchMax, pitchAve);
    //        Serial.print(message);
    //        sprintf(message, "rollMin: %4.1f     rollMax: %4.1f     rollAve: %5.2f   ", rollMin, rollMax, rollAve);
    //        Serial.print(message);
    //        sprintf(message, "yawMin: %4.1f     yawMax: %4.1f     yawAve: %5.1f\n", yawMin, yawMax, yawAve);
    //        Serial.print(message);
  }
}



/*****************************************************************************-
 *    setDrift()  Called to add the last 1/2 sec of drift values.
 *****************************************************************************/
void setDrift(float pitchAve, float rollAve, float yawAve) {
  static const int AVE_SIZE =  20;      // Equals 10 seconds of measurements
  static float pitchAveArray[AVE_SIZE];
  static float rollAveArray[AVE_SIZE];
  static float yawAveArray[AVE_SIZE];
  static int avePtr = 0;

  float sumPitchAve = 0.0;
  float sumRollAve = 0.0;
  float sumYawAve = 0.0;

  pitchAveArray[avePtr] = pitchAve;
  rollAveArray[avePtr] = rollAve;
  yawAveArray[avePtr] = yawAve;

  ++avePtr;
  avePtr = avePtr % AVE_SIZE;
  if (aveTotal < avePtr) aveTotal = avePtr;

  for (int i = 0; i < aveTotal; i++) {
    sumPitchAve += pitchAveArray[i];
    sumRollAve += rollAveArray[i];
    sumYawAve += yawAveArray[i];
  }
  float avePitchDrift = sumPitchAve / aveTotal;
  float aveRollDrift = sumRollAve / aveTotal;
  float aveYawDrift = sumYawAve / aveTotal;
  timeDriftPitch = avePitchDrift;
  timeDriftRoll = aveRollDrift;
  timeDriftYaw = aveYawDrift;
}



/*****************************************************************************-
     MadgwickQuaternionUpdate()

     Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
   (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
    device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
 *****************************************************************************/

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx = 0.0, gbiasy = 0.0, gbiasz = 0.0;        // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 - (beta * hatDot1)) * deltat;
  q2 += (qDot2 - (beta * hatDot2)) * deltat;
  q3 += (qDot3 - (beta * hatDot3)) * deltat;
  q4 += (qDot4 - (beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

/*****************************************************************************-
     MahonyAHRSupdateIMU()

     From: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

 *****************************************************************************/


#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki


void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0 *= recipNorm;
  q[1] = q1 *= recipNorm;
  q[2] = q2 *= recipNorm;
  q[3] = q3 *= recipNorm;
} // End MahonyAHRSupdateIMU()



//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
