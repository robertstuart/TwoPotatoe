/*****************************************************************************-
 *                        Skate.ino
 *              Routines to balance in varied terrain
 *****************************************************************************/
#define CAMPOINTS_PER_FOOT   20.0
#define CAM_ARRAY_SIZE     500
#define CAM_SETBACK           1.5  // Distance of measured area in front of wheels
#define ILLEGAL_CAM        9999.9F

struct camPoint {
  float feet;
  float slope;
};
struct camPoint camArray[CAM_ARRAY_SIZE];  // Circular buffer


/*****************************************************************************-
 *  logCam() Log the slope at the position so that TwoPotatoe will know the
 *           slope when it reaches that point.  The slope is the pitch angle
 *           from the camera minus the gaPitch.  Positive slope = uphill.
 *****************************************************************************/
void logCam() {
  float camFeet = feetPosition + CAM_SETBACK;  // Point at which slope is measured
  int camIndex = (int) (camFeet * CAMPOINTS_PER_FOOT);
  camIndex = modCam(camIndex);  // This is the place we will store value.
  struct camPoint cp;
  cp.feet = feetPosition;
  cp.slope = camPitch - gaPitch;
  camArray[camIndex] = cp;

  // Set nearby points if they aren't set nearby.
  for (int i = -4; i ++; i <= 4) {  // +- 4 (3") should be plenty
    int camIndexFix = modCam(camIndex   + i);
    int f = camArray[camIndexFix].feet;
    if (abs(f - feetPosition) > 1.0) {  // further than a foot?
      camArray[camIndexFix] = cp;
    }
  }
}



/*****************************************************************************-
 *  getCamSlope() Return the pitch at this current location.
 *****************************************************************************/
float getCamSlope() {
  int camIndex = (int) (feetPosition * CAMPOINTS_PER_FOOT);
  camIndex = modCam(camIndex);
  return camArray[camIndex].slope;
}


/*****************************************************************************-
 *  modCam() Return array index that wraps around and is non-negative
 *****************************************************************************/
int modCam(int a) {
   return ((a % CAM_ARRAY_SIZE) + CAM_ARRAY_SIZE) % CAM_ARRAY_SIZE;
}


/*****************************************************************************-
 *  camInit() fill wioth illegal values
 *****************************************************************************/
void camInit() {
  struct camPoint cp = {ILLEGAL_CAM, ILLEGAL_CAM};
  for (int i = 0; i < CAM_ARRAY_SIZE; i++) {
    camArray[i] = cp;
  }
}



/*****************************************************************************-
 *  isIllegalCam()
 *****************************************************************************/
bool isIllegalCam(float slope) {
  return slope > (ILLEGAL_CAM - 1.0);
}
