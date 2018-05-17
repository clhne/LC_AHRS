#include "SignificantMotion.h"
sig_motion_t sigmotion;
BSX_S8 sigmotion_set_opmode(BSX_U16 mode) {
  sigmotion.opmode = mode;
  if (mode) {
    sigmotion_reset();
    sigmotion_init();
  } else {
    sigmotion_reset();
  }
  return 0;
}

BSX_S8 sigmotion_run(ts_dataxyzs32 accelsample) {
  BSX_S32 delta_ax, delta_ay, delta_az;
  if (!sigmotion.refaccel[0] && !sigmotion.refaccel[1] && !sigmotion.refaccel[2]) {
    sigmotion.refaccel[0] = accelsample.x;
    sigmotion.refaccel[1] = accelsample.y;
    sigmotion.refaccel[2] = accelsample.z;
  }
  if (!sigmotion.eventstoskip) {
    delta_ax = abs(accelsample.x - sigmotion.refaccel[0]);
    delta_ay = abs(accelsample.y - sigmotion.refaccel[1]);
    delta_az = abs(accelsample.z - sigmotion.refaccel[2]);
    if (delta_ax > (BSX_S32)sigmotion.accelthresholdmax
     || delta_ay > (BSX_S32)sigmotion.accelthresholdmax
     || delta_az > (BSX_S32)sigmotion.accelthresholdmax) {
       if (sigmotion.movementdetectedrecently == 1) {
         sigmotion.refaccel[0] = accelsample.x;
         sigmotion.refaccel[1] = accelsample.y;
         sigmotion.refaccel[2] = accelsample.z;
         sigmotion.sigmotion_status = 1;
         sigmotion.movementdetectedrecently = 0;
         sigmotion.retrysamplecount = 0;
         return 0;
       } else {
         sigmotion.movementdetectedrecently = 1;
         sigmotion.eventstoskip = sigmotion.samplestoskip;
       }
    } else {
      if (sigmotion.movementdetectedrecently == 1) {
        sigmotion.retrysamplecount++;
        if (sigmotion.maxsampleforseconddetection < sigmotion.retrysamplecount) {
          sigmotion.retrysamplecount = 0;
          sigmotion.movementdetectedrecently = 0;
        }
      }
    }
    sigmotion.sigmotion_status = 0;
  } else {
    if (sigmotion.eventstoskip == 1) {
      sigmotion.refaccel[0] = accelsample.x;
      sigmotion.refaccel[1] = accelsample.y;
      sigmotion.refaccel[2] = accelsample.z;
      sigmotion.retrysamplecount = 0;
    }
    delta_ax = abs(accelsample.x - sigmotion.refaccel[0]);
    delta_ay = abs(accelsample.y - sigmotion.refaccel[1]);
    delta_az = abs(accelsample.z - sigmotion.refaccel[2]);
    if (delta_ax >= (BSX_S32)sigmotion.accelthresholdmin
     || delta_ay >= (BSX_S32)sigmotion.accelthresholdmin
     || delta_az >= (BSX_S32)sigmotion.accelthresholdmin) {
       sigmotion.nomotioncounter = 0;
    } else {
      if ((BSX_U8)(sigmotion.previouscount + 1) == sigmotion.eventstoskip) {
        ++sigmotion.nomotioncounter;
      }
      sigmotion.previouscount = (BSX_U8)sigmotion.eventstoskip;
    }
    if (sigmotion.nomotioncounter <= 1) {
      --sigmotion.eventstoskip;
      sigmotion.sigmotion_status = 0;
    }
    sigmotion.movementdetectedrecently = 0;
    sigmotion.eventstoskip = 0;
  }
  return 0;
}

BSX_S8 sigmotion_reset() {
  sigmotion.refaccel[0] = 0;
  sigmotion.refaccel[1] = 0;
  sigmotion.refaccel[2] = 0;
  sigmotion.eventstoskip = 0;
  sigmotion.movementdetectedrecently = 0;
  sigmotion.retrysamplecount = 0;
  sigmotion.sigmotion_status = 0;
  return 0;
}

BSX_S8 sigmotion_init() {
  sigmotion.accelthresholdmax = 1500;
  sigmotion.accelthresholdmin = 200;
  sigmotion.samplestoskip = 30;
  sigmotion.maxsampleforseconddetection = 10;
  sigmotion.sigmotion_status = 0;
  sigmotion.nomotioncounter = 0;
  sigmotion.previouscount = 0;
  return 0;
}

BSX_S8 sigmotion_get_status(BSX_U16 *status) {
  *status = sigmotion.sigmotion_status;
  return 0;
}

BSX_S8 sigmotion_get_opmode(BSX_U16 *mode) {
  *mode = sigmotion.opmode;
  return 0;
}
