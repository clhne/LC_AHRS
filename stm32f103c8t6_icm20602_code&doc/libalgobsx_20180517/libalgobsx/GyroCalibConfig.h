#ifndef __GYROCALIBCONFIG_H__
#define __GYROCALIBCONFIG_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_U8 DynamicDetectionThreshold;
  BSX_U8 NoDynamicDetectionTime;
} ts_MagObserv;
typedef struct {
  BSX_U16 DetectionThreshold;
  BSX_S8 DetectionTime;
} ts_ShakeDetect;
typedef struct {
  ts_ShakeDetect S_ShakeDetection;
  BSX_U8 DynamicDetectionThreshold;
  BSX_U8 NoDynamicDetectionTime;
} ts_GyroObserv;
typedef struct {
  BSX_U8 OffsetThreshold;
  BSX_U8 Step;
} ts_GyroOffsetThreshold;
typedef struct {
  BSX_U8 opMode;
  ts_MagObserv S_MagObserv;
  ts_GyroObserv S_GyroObserv;
  ts_GyroOffsetThreshold S_GyroOffsetThreshold;
} ts_gyrocalibconfig;
BSX_S8 gyrocalibconfig_setDefaultParam(ts_gyrocalibconfig *gyroCalibConfig);
#endif