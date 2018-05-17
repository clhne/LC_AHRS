#include <math.h>
#include "BsxLibraryConfiguration.h"
#include "GyroCalbration.h"
#include "MathOperations.h"

ts_GyroCalib S_GyroCalib;
BSX_S8 GYROCALIB_UpdateOffsetStepwise(BSX_F32 *v_offsetValue_f32r, BSX_F32 v_meanValue_f32r) {
  BSX_F32 new_offsetValue;
  BSX_F32 old_offsetValue = *v_offsetValue_f32r;
  BSX_F32 diff_offsetValue_meanValue = *v_offsetValue_f32r - v_meanValue_f32r;
  if (fabsf(diff_offsetValue_meanValue) >= (BSX_F32)S_GyroCalib.S_GyroOffsetThres.V_Step_U8R) {
    if (diff_offsetValue_meanValue > -(BSX_F32)S_GyroCalib.S_GyroOffsetThres.V_Step_U8R) {
      new_offsetValue = old_offsetValue - S_GyroCalib.S_GyroOffsetThres.V_Step_U8R;
    } else {
      new_offsetValue = old_offsetValue + S_GyroCalib.S_GyroOffsetThres.V_Step_U8R;
    }
  } else {
    new_offsetValue = v_meanValue_f32r;
  }
  *v_offsetValue_f32r = new_offsetValue * 0.1f + old_offsetValue * 0.9f;
  return 0;
}

BSX_S8 GYROCALIB_SetShakeDetectionSettings(BSX_U16 v_threshold_u16r, BSX_S8 v_time_s8r) {
  GYROOBSERVER_SetShakeDetectionParam(&S_GyroCalib.S_GyroObserver, v_threshold_u16r, v_time_s8r);
  return 0;
}

BSX_S8 GYROCALIB_SetOperationMode(BSX_U8 v_opmode_u8r) {
  S_GyroCalib.V_Opmode_U8R = v_opmode_u8r;
  return 0;
}

BSX_S8 GYROCALIB_SetGyroOffsetThreshold(BSX_U8 v_offsetThres_u8r, BSX_U8 v_step_u8r) {
  S_GyroCalib.S_GyroOffsetThres.V_Max_U8R = v_offsetThres_u8r;
  S_GyroCalib.S_GyroOffsetThres.V_Step_U8R = v_step_u8r;
  return 0;
}

BSX_S8 GYROCALIB_SetGyroObservNoDynamicDetectionTime(BSX_U8 v_noDynDetTime_u8r) {
  GYROOBSERVER_SetNoDynamicDetectionTime(&S_GyroCalib.S_GyroObserver, v_noDynDetTime_u8r);
  return 0;
}

BSX_S8 GYROCALIB_SetGyroObservDynamicDetectionThreshold(BSX_U8 v_threshold_u8r) {
  GYROOBSERVER_SetDynamicDetectionThreshold(&S_GyroCalib.S_GyroObserver, v_threshold_u8r);
  return 0;
}

BSX_S8 GYROCALIB_SetGyroCalibrationSettings(BSX_U8 *p_gyrocalibconfig, BSX_U32 *p_length) {
  BSX_U16 shake_det_threshold;
  GYROCALIB_SetGyroObservDynamicDetectionThreshold(p_gyrocalibconfig[*p_length]);
  p_length++;
  GYROCALIB_SetGyroObservNoDynamicDetectionTime(p_gyrocalibconfig[*p_length]);
  p_length++;
  GYROCALIB_SetGyroOffsetThreshold(p_gyrocalibconfig[*p_length], p_gyrocalibconfig[*p_length + 1]);
  p_length++;
  p_length++;
  bsx_copyBytefromMemory((BSX_U8 *)&shake_det_threshold, &p_gyrocalibconfig[*p_length], 2);
  p_length++;
  p_length++;
  GYROCALIB_SetShakeDetectionSettings(shake_det_threshold, p_gyrocalibconfig[*p_length]);
  p_length++;
  return 0;
}

BSX_S8 GYROCALIB_SetCalibParam(ts_dataxyzf32 s_calibParam) {
  S_GyroCalib.S_GyroOffset_F32R.x = s_calibParam.x;
  S_GyroCalib.S_GyroOffset_F32R.y = s_calibParam.y;
  S_GyroCalib.S_GyroOffset_F32R.z = s_calibParam.z;
  return 0;
}

BSX_S8 GYROCALIB_SetAccuracy(BSX_U8 v_accuracy_u8r) {
  S_GyroCalib.V_Accuracy_U8R = v_accuracy_u8r;
  return 0;
}

BSX_S8 GYROCALIB_ResetObservationEngines() {
  GYROOBSERVER_Reset(&S_GyroCalib.S_GyroObserver);
  return 0;
}

BSX_S8 GYROCALIB_ResetGyroOffset() {
  S_GyroCalib.S_GyroOffset_F32R.x = 0.0f;
  S_GyroCalib.S_GyroOffset_F32R.y = 0.0f;
  S_GyroCalib.S_GyroOffset_F32R.z = 0.0f;
  S_GyroCalib.V_Accuracy_U8R = 0;
  return 0;
}

BSX_S8 GYROCALIB_Reset() {
  GYROCALIB_ResetGyroOffset();
  GYROCALIB_ResetObservationEngines();
  return 0;
}

BSX_S8 GYROCALIB_Init(BSX_U8 v_gyrohwid_u8r) {
  S_GyroCalib.V_Opmode_U8R = 3;
  S_GyroCalib.S_GyroOffsetThres.V_Step_U8R = 4;
  S_GyroCalib.V_CalibStatus_U8R = 0;
  S_GyroCalib.S_GyroOffsetThres.V_Max_U8R = 163;
  S_GyroCalib.V_ssdmethod = 1;
  GYROCALIB_ResetGyroOffset();
  GYROOBSERVER_Init(&S_GyroCalib.S_GyroObserver, v_gyrohwid_u8r);
  GYROCALIB_SetGyroObservDynamicDetectionThreshold(19);
  GYROCALIB_SetGyroObservNoDynamicDetectionTime(50);
  S_GyroCalib.V_calibrationMethod = 1;
  GYROCALIB_ResetObservationEngines();
  return 0;
}

BSX_S8 GYROCALIB_GetVersion(ts_version *p_swid) {
  p_swid->major = 0;
  p_swid->minor = 0;
  p_swid->minorbugFix = 0;
  p_swid->majorbugFix = 0;
  return 0;
}

BSX_S8 GYROCALIB_GetShakeStatus(BSX_U8 *v_shakeStatus_u8r) {
  GYROOBSERVER_GetShakeStatus(&S_GyroCalib.S_GyroObserver, v_shakeStatus_u8r);
  return 0;
}

BSX_S8 GYROCALIB_GetOperationMode(BSX_U8 *v_opmode_u8r) {
  *v_opmode_u8r = S_GyroCalib.V_Opmode_U8R;
  return 0;
}

BSX_S8 GYROCALIB_GetGyroOffsetThreshold(BSX_U8 *v_offsetThres_u8r, BSX_U8 *v_step_u8r) {
  *v_offsetThres_u8r = S_GyroCalib.S_GyroOffsetThres.V_Max_U8R;
  *v_step_u8r = S_GyroCalib.S_GyroOffsetThres.V_Step_U8R;
  return 0;
}

BSX_S8 GYROCALIB_GetGyroObservNoDynamicDetectionTime(BSX_U8 *v_noDynDetTime_u8r) {
  GYROOBSERVER_GetNoDynamicDetectionTime(&S_GyroCalib.S_GyroObserver, v_noDynDetTime_u8r);
  return 0;
}

BSX_S8 GYROCALIB_GetGyroObservDynamicDetectionThreshold(BSX_U8 *v_threshold_u8r) {
  GYROOBSERVER_GetDynamicDetectionThreshold(&S_GyroCalib.S_GyroObserver, v_threshold_u8r);
  return 0;
}

BSX_S8 GYROCALIB_GetCalibStatus(BSX_U8 *v_CalibStatus_u8r) {
  *v_CalibStatus_u8r = S_GyroCalib.V_CalibStatus_U8R;
  return 0;
}

BSX_S8 GYROCALIB_GetCalibParam(ts_dataxyzf32 *p_calibParam) {
  p_calibParam->x = S_GyroCalib.S_GyroOffset_F32R.x;
  p_calibParam->y = S_GyroCalib.S_GyroOffset_F32R.y;
  p_calibParam->z = S_GyroCalib.S_GyroOffset_F32R.z;
  return 0;
}

BSX_S8 GYROCALIB_GetAccuracy(BSX_U8 *v_accuracy_u8r) {
  *v_accuracy_u8r = S_GyroCalib.V_Accuracy_U8R;
  return 0;
}

BSX_S8 GYROCALIB_DoStep(BSX_U8 v_accelDynStatus_u8r, BSX_U8 v_magDynStatus_u8r, ts_dataxyzf32 *p_gyroData, BSX_U8 v_doAccStepTick_u8r, BSX_U8 v_doGyroStepTick_u8r, BSX_U8 v_doMagnStepTick_u8r) {
  BSX_S32 noDynDetTime;
  ts_dataxyzf32 gyroMeanXYZ;
  S_GyroCalib.V_CalibStatus_U8R = 0;
  switch (S_GyroCalib.V_Opmode_U8R) {
  case 1:
    if (v_doGyroStepTick_u8r) {
      GYROOBSERVER_DoStep(p_gyroData, &S_GyroCalib.S_GyroObserver);
    }
    if (S_GyroCalib.S_GyroObserver.V_DynamicStatus_U8R) {
      return 0;
    }
    break;
  case 2:
    if (v_doGyroStepTick_u8r) {
      GYROOBSERVER_DoStep(p_gyroData, &S_GyroCalib.S_GyroObserver); 
    }
    if (!v_doAccStepTick_u8r || v_accelDynStatus_u8r || S_GyroCalib.S_GyroObserver.V_DynamicStatus_U8R) {
      return 0;
    }
    break;
  case 3:
    if (v_doGyroStepTick_u8r) {
      GYROOBSERVER_DoStep(p_gyroData, &S_GyroCalib.S_GyroObserver);
    }
    if (!v_doMagnStepTick_u8r || v_magDynStatus_u8r || v_accelDynStatus_u8r || S_GyroCalib.S_GyroObserver.V_DynamicStatus_U8R) {
      return 0;
    }
    break;
  default:
    return 0;
  }
  noDynDetTime = S_GyroCalib.S_GyroObserver.V_NoDynDetTime_U8R;
  if (S_GyroCalib.S_GyroObserver.V_NoDynDetTime_U8R >= 25) {
    noDynDetTime = 25;
  }
  gyroMeanXYZ.x = MATH_Mean(S_GyroCalib.S_GyroObserver.S_GyroRatesBuffer.Wx_buf, 0, noDynDetTime);
  gyroMeanXYZ.y = MATH_Mean(S_GyroCalib.S_GyroObserver.S_GyroRatesBuffer.Wy_buf, 0, noDynDetTime);
  gyroMeanXYZ.z = MATH_Mean(S_GyroCalib.S_GyroObserver.S_GyroRatesBuffer.Wz_buf, 0, noDynDetTime);
  S_GyroCalib.S_GyroObserver.V_DynDetCounter_U8R = 0;
  if (fabsf(gyroMeanXYZ.x) < (BSX_F32)S_GyroCalib.S_GyroOffsetThres.V_Max_U8R 
   && fabsf(gyroMeanXYZ.y) < (BSX_F32)S_GyroCalib.S_GyroOffsetThres.V_Max_U8R
   && fabsf(gyroMeanXYZ.z) < (BSX_F32)S_GyroCalib.S_GyroOffsetThres.V_Max_U8R
   ) {
     if (S_GyroCalib.V_Accuracy_U8R == 1
      || S_GyroCalib.V_Accuracy_U8R == 2
      || S_GyroCalib.V_Accuracy_U8R == 3) {
       GYROCALIB_UpdateOffsetStepwise(&S_GyroCalib.S_GyroOffset_F32R.x, gyroMeanXYZ.x);
       GYROCALIB_UpdateOffsetStepwise(&S_GyroCalib.S_GyroOffset_F32R.y, gyroMeanXYZ.y);
       GYROCALIB_UpdateOffsetStepwise(&S_GyroCalib.S_GyroOffset_F32R.z, gyroMeanXYZ.z);
       if (S_GyroCalib.V_Accuracy_U8R == 3) {
         S_GyroCalib.V_Accuracy_U8R = 1;
       } else {
         S_GyroCalib.V_Accuracy_U8R++;
       }
     } else {
       S_GyroCalib.S_GyroOffset_F32R.x = gyroMeanXYZ.x;
       S_GyroCalib.S_GyroOffset_F32R.y = gyroMeanXYZ.y;
       S_GyroCalib.S_GyroOffset_F32R.z = gyroMeanXYZ.z;
       S_GyroCalib.V_Accuracy_U8R = 1;
     }
  }
  S_GyroCalib.V_CalibStatus_U8R = 1;
  return 0;
}