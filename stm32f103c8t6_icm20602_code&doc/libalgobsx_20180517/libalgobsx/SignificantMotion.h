#ifndef __SIGNIFICANTMOTION_H__
#define __SIGNIFICANTMOTION_H__
#include "BsxLibraryDataTypes.h"
typedef struct{
  BSX_S32 refaccel[3];
  BSX_U16 accelthresholdmax;
  BSX_U16 accelthresholdmin;
  BSX_U16 samplestoskip;
  BSX_U16 maxsampleforseconddetection;
  BSX_U16 eventstoskip;
  BSX_U16 retrysamplecount;
  BSX_U16 sigmotion_status;
  BSX_U16 opmode;
  BSX_S8 movementdetectedrecently;
  BSX_S8 nomotioncounter;
  BSX_U8 previouscount;
} sig_motion_t;
BSX_S8 sigmotion_set_opmode(BSX_U16 mode);
BSX_S8 sigmotion_run(ts_dataxyzs32 accelsample);
BSX_S8 sigmotion_reset();
BSX_S8 sigmotion_init();
BSX_S8 sigmotion_get_status(BSX_U16 *status);
BSX_S8 sigmotion_get_opmode(BSX_U16 *mode);
#endif