/*****************************************************************************-
 *                        Skate.ino
 *              Routines to balance in varied terrain
 *****************************************************************************/
#define CAMPOINTS_PER_FOOT   20.0
#define CAM_POINTS_SIZE     500
#define CAM_SETBACK           1.5  // Distance of measured area in front of wheels
#define ILLEGAL_CAM        9999.9F

struct camPoint {
  float feet;
  float slope;
};
struct camPoint camArray[CAM_POINTS_SIZE];  // Circular buffer


/*****************************************************************************-
 *  logCam() Log the slope at the position so that TwoPotatoe will know the
 *           slope when it reaches that point.  The slope is the pitch angle
 *           from the camera minus the maPitch.  Positive slope = uphill.
 *****************************************************************************/
void logCam() {
  float camFeet = feetPosition + CAM_SETBACK;  // Point at which slope is measured
  int camIndex = (int) (camFeet * CAMPOINTS_PER_FOOT);
  camIndex = modCam(camIndex);  // This is the place we will store value.
  struct camPoint cp;
  cp.feet = feetPosition;
  cp.slope = camPitch - maPitch;
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
  struct camPoint cp = camArray[camIndex];
  if (cp.slope >= (ILLEGAL_CAM))  return maPitch;
  else return cp.slope;
}


/*****************************************************************************-
 *  modCam() Return array index that wraps around
 *****************************************************************************/
int modCam(int a) {
   return ((a % CAM_POINTS_SIZE) + CAM_POINTS_SIZE) % CAM_POINTS_SIZE;
}


/*****************************************************************************-
 *  camInit() fill wioth illegal values
 *****************************************************************************/
void camInit() {
  struct camPoint cp = {ILLEGAL_CAM - 1.0, ILLEGAL_CAM - 1.0};
  for (int i = 0; i < CAM_POINTS_SIZE; i++) {
    camArray[i] = cp;
  }
}
