#ifndef __GENERICGYROSPEC_H__
#define __GENERICGYROSPEC_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_U8 name;
  BSX_U8 opMode;
  BSX_U8 range;
  BSX_U8 dataRate;
  BSX_U8 maxDatarate;
  BSX_S16 measCurrent;
  BSX_U8 resBits;
  BSX_U8 version;
  BSX_S8 vendor;
} ts_gyrosensorspec;
BSX_S8 genericgyrospec_setSensorSpec(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *gyrodata);
BSX_S8 genericgyrospec_setRange(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 userrange);
BSX_S8 genericgyrospec_setOpMode(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 useropmode);
BSX_S8 genericgyrospec_setDatarate(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 userdatarate);
BSX_S8 genericgyrospec_getVersion(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *version);
BSX_S8 genericgyrospec_getVendor(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *vendor);
BSX_S8 genericgyrospec_getType(BSX_U8 *type);
BSX_S8 genericgyrospec_getResolution(BSX_F32 *res);
BSX_S8 genericgyrospec_getRange(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *range);
BSX_S8 genericgyrospec_getPower(ts_gyrosensorspec *gyroSensorSpec, BSX_S16 *powerConsumption);
BSX_S8 genericgyrospec_getOpMode(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *opmode);
BSX_S8 genericgyrospec_getName(ts_gyrosensorspec *gyroSensorSpec, BSX_S8 *name);
BSX_S8 genericgyrospec_getMinDelay(ts_gyrosensorspec *gyroSensorSpec, BSX_F32 *minDelay);
BSX_S8 genericgyrospec_getMaximumRange(ts_gyrosensorspec *gyroSensorSpec, BSX_S16 *maxRange);
BSX_S8 genericgyrospec_getDatarate(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *datarate);
#endif