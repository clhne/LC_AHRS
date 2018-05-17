#include <math.h>
#include "MathOperations.h"
#include "flip_gesture.h"

flipgesture_t flipgesture;
BSX_S8 flipgesture_set_flipstate_detecttime(BSX_U16 det_time, BSX_U16 ref_time) {
  if (det_time <= 2000) {
    flipgesture.detect_time = det_time;
  }
  if ( ref_time <= 1000 && det_time <= 2000 ) {
    flipgesture.ref_time = ref_time;
    flipgesture.flipcount_thres = (BSX_U16)ceilf((BSX_F32)det_time / (BSX_F32)ref_time);
  }
  flipgesture_reset();
  return 0;
}

BSX_S8 flipgesture_set_flipangle(flip_t flipangle) {
  if (flipangle.facedown <= 80.0 && flipangle.facedown >= 10.0) {
    flipgesture.flip_thres.facedown = -sinf((90.0f - flipangle.facedown) * 0.0175f);
  }
  if (flipangle.faceup <= 170.0 && flipangle.faceup >= 100.0) {
    flipgesture.flip_thres.faceup = -sinf((90.0f - flipangle.faceup) * 0.0175f);
  }
  flipgesture_reset();
  return 0;
}

BSX_S8 flipgesture_run(ts_dataxyzf32 accdata, BSX_U8 onegflag) {
  if (onegflag) {
    flipgesture.flipcount.faceup = 0;
    flipgesture.flipcount.facedown = 0;
    flipgesture.flipcount.neutral = 0;
  } else {
    if (accdata.z > flipgesture.flip_thres.faceup) {
      if (flipgesture.flipcount_thres > flipgesture.flipcount.faceup) {
        flipgesture.flipcount.faceup++;
      }
      flipgesture.flipcount.facedown = 0;
      flipgesture.flipcount.neutral = 0;
    } else if (accdata.z < flipgesture.flip_thres.facedown) {
      if (flipgesture.flipcount_thres > flipgesture.flipcount.facedown) {
        flipgesture.flipcount.facedown++;
      }
      flipgesture.flipcount.faceup = 0;
      flipgesture.flipcount.neutral = 0;
    } else if (accdata.z > flipgesture.flip_thres.facedown + flipgesture.hyst_thres 
      && accdata.z < flipgesture.flip_thres.faceup - flipgesture.hyst_thres) {
      if (flipgesture.flipcount_thres > flipgesture.flipcount.neutral) {
        flipgesture.flipcount.neutral++;
      }
      flipgesture.flipcount.faceup = 0;
      flipgesture.flipcount.facedown = 0;
    }
  }
  if (flipgesture.flipcount.faceup >= flipgesture.flipcount_thres) {
    flipgesture.flip_status = 1;
  }
  if (flipgesture.flipcount.facedown >= flipgesture.flipcount_thres) {
    flipgesture.flip_status = 2;
  }
  if (flipgesture.flipcount.neutral >= flipgesture.flipcount_thres) {
    flipgesture.flip_status = 3;
  }
  return 0;
}

BSX_S8 flipgesture_reset() {
  flipgesture.flipcount.facedown = 0;
  flipgesture.flipcount.faceup = 0;
  flipgesture.flipcount.neutral = 0;
  flipgesture.flip_status = 0;
  return 0;
}

BSX_S8 flipgesture_init() {
  flipgesture.flip_thres.facedown = -0.9659f;
  flipgesture.hyst_thres = 0.025f;
  flipgesture.flip_thres.faceup = 0.2588f;
  flipgesture_reset();
  flipgesture.flipcount_thres = 5;
  flipgesture.ref_time = 100;
  flipgesture.detect_time = 500;
  return 0;
}

BSX_S8 flipgesture_get_flipstatus(BSX_U8 *flip_status) {
  *flip_status = flipgesture.flip_status;
  return 0;
}

BSX_S8 flipgesture_get_flipstate_detecttime(BSX_U16 *time) {
  *time = flipgesture.detect_time;
  return 0;
}

BSX_S8 flipgesture_get_flipangle(flip_t *flipangle) {
  flipangle->facedown = MATH_Round(asinf(flipgesture.flip_thres.facedown) * 57.296f + 90.0f);
  flipangle->faceup = MATH_Round(asinf(flipgesture.flip_thres.faceup) * 57.296f + 90.0f);
  return 0;
}
