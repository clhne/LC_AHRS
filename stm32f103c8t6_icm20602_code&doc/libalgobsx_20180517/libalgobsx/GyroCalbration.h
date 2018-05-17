#ifndef __BSX_GYROCALBRATION_H__
#define __BSX_GYROCALBRATION_H__
#include "BsxLibraryDataTypes.h"
#include "BSX_ObserverGyro.h"
typedef struct{
  BSX_U8 V_Max_U8R;
  BSX_U8 V_Step_U8R;
}ts_OffsetThresh;
typedef struct{
  ts_dataxyzf32 S_GyroOffset_F32R;
  ts_OffsetThresh S_GyroOffsetThres;
  ts_GYROBSERVER S_GyroObserver;
  ts_dataxyzf32 S_Offset_F32R;
  BSX_F32 V_MagnScalingFactor_F32R;
  BSX_F32 V_offset_smoothcoeff;
  BSX_U8 V_Opmode_U8R;
  BSX_U8 V_CalibStatus_U8R;
  BSX_U8 V_Accuracy_U8R;
  BSX_U8 V_calibrationMethod;
  BSX_U8 V_ssdmethod;
} ts_GyroCalib;
#endif