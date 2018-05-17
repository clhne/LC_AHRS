#ifndef __GENERICACCSPEC_H__
#define __GENERICACCSPEC_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_U8 name;
  BSX_U8 opMode;
  BSX_U8 range;
  BSX_U8 dataRate;
  BSX_U8 maxDatarate;
  BSX_S16 measCurrent;
  BSX_U8 resBits;
  BSX_S16 offErr;
  BSX_S16 sensErr;
  BSX_S16 noise1s;
  BSX_S16 gVecErrMax;
  BSX_S16 gVecErrMin;
  BSX_F32 noise;
  BSX_U8 noisefactorarray[9];
  BSX_U8 version;
  BSX_U8 vendor;
} ts_accsensorspec;
BSX_S8 genericaccspec_setSensorSpec(ts_accsensorspec *accSensorSpec, BSX_U8* accdata);
BSX_S8 genericaccspec_setRange(ts_accsensorspec *accSensorSpec, BSX_U8 range);
BSX_S8 genericaccspec_setOpMode(ts_accsensorspec *accSensorSpec, BSX_S8 opmode);
BSX_S8 genericaccspec_setDatarate(ts_accsensorspec *accSensorSpec, BSX_U8 datarate);
BSX_S8 genericaccspec_getVersion(ts_accsensorspec *accSensorSpec, BSX_U8 *version);
BSX_S8 genericaccspec_getVendor(ts_accsensorspec *accSensorSpec, BSX_U8 *vendor);
BSX_S8 genericaccspec_getType(BSX_U8 *type);
BSX_S8 genericaccspec_getSensorRmsNoise(BSX_F32 *rmsNoise);
BSX_S8 genericaccspec_getSensitivityErr(ts_accsensorspec *accSensorSpec, BSX_S16 *maxSensErr);
BSX_S8 genericaccspec_getResolution(BSX_F32 *res);
BSX_S8 genericaccspec_getRange(ts_accsensorspec *accSensorSpec, BSX_U8 *range);
BSX_S8 genericaccspec_getPower(ts_accsensorspec *accSensorSpec, BSX_S16 *powerConsumption);
BSX_S8 genericaccspec_getOpMode(ts_accsensorspec *accSensorSpec, BSX_U8 *opmode);
BSX_S8 genericaccspec_getOffsetErr(ts_accsensorspec *accSensorSpec, BSX_S16 *maxOffErr);
BSX_U8 genericaccspec_getNoiseFactor(ts_accsensorspec *accSensorSpec, BSX_U8 factor);
BSX_S8 genericaccspec_getNoise(ts_accsensorspec *accSensorSpec, BSX_F32 *noise);
BSX_S8 genericaccspec_getName(ts_accsensorspec *accSensorSpec, BSX_S8 *name);
BSX_S8 genericaccspec_getMinDelay(ts_accsensorspec *accSensorSpec, BSX_F32 *minDelay);
BSX_S8 genericaccspec_getMaximumRange(ts_accsensorspec *accSensorSpec, BSX_S16 *maxRange);
BSX_S8 genericaccspec_getGVectorErrors(ts_accsensorspec *accSensorSpec, BSX_S16 *gVecErrMax, BSX_S16 *gVecErrMin);
BSX_S8 genericaccspec_getDatarate(ts_accsensorspec *accSensorSpec, BSX_U8 *datarate);
#endif