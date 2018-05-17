#ifndef __GENERICMAGSPEC_H__
#define __GENERICMAGSPEC_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  ts_dataxyz range;
  BSX_F32 res;
  BSX_F32 rmsNoise;
  BSX_F32 noise;
  BSX_S16 measCurrent;
  BSX_U8 name;
  BSX_U8 opMode;
  BSX_U8 dataRate;
  BSX_U8 maxDatarate;
  BSX_U8 noisefactorarray[9];
  BSX_U8 version;
  BSX_U8 vendor;
} ts_magsensorspec;
BSX_U8 genericmagspec_setSensorSpec(ts_magsensorspec *magSensorSpec, BSX_U8* magdata);
BSX_S8 genericmagspec_setRange(ts_magsensorspec *magSensorSpec, ts_dataxyz range);
BSX_S8 genericmagspec_setOpMode(ts_magsensorspec *magSensorSpec, BSX_U8 opmode);
BSX_S8 genericmagspec_setDatarate(ts_magsensorspec *magSensorSpec, BSX_U8 datarate);
BSX_S8 genericmagspec_getVersion(ts_magsensorspec *magSensorSpec, BSX_U8 *version);
BSX_S8 genericmagspec_getVendor(ts_magsensorspec *magSensorSpec, BSX_U8 *vendor);
BSX_S8 genericmagspec_getType(BSX_U8 *type);
BSX_S8 genericmagspec_getSensorRmsNoise(ts_magsensorspec *magSensorSpec, BSX_F32 *rmsNoise);
BSX_S8 genericmagspec_getResolution(ts_magsensorspec *magSensorSpec, BSX_F32 *res);
BSX_S8 genericmagspec_getRange(ts_magsensorspec *magSensorSpec, ts_dataxyz *range);
BSX_S8 genericmagspec_getPower(ts_magsensorspec *magSensorSpec, BSX_F32 *powerConsumption);
BSX_S8 genericmagspec_getOpMode(ts_magsensorspec *magSensorSpec, BSX_U8 *opMode);
BSX_S8 genericmagspec_getNoiseFactor(ts_magsensorspec *magSensorSpec, BSX_U8 factor);
BSX_S8 genericmagspec_getNoise(ts_magsensorspec *magSensorSpec, BSX_F32 *noise);
BSX_S8 genericmagspec_getName(ts_magsensorspec *magSensorSpec, BSX_S8 *name);
BSX_S8 genericmagspec_getMinDelay(ts_magsensorspec *magSensorSpec, BSX_F32 *minDelay);
BSX_S8 genericmagspec_getMaximumRange(ts_magsensorspec *magSensorSpec, ts_dataxyz *maxRange);
BSX_S8 genericmagspec_getDatarate(ts_magsensorspec *magSensorSpec, BSX_U8 *datarate);
#endif


