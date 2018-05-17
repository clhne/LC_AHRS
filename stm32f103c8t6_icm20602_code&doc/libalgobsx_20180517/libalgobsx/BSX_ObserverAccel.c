#include <math.h>
#include "BSX_ObserverAccel.h"
#include "MathOperations.h"
BSX_S8 ACCELOBSERVER_SetParameterSettings(ts_AccelObserver *s_AccelObserver, BSX_U8 v_threshold_u8r, BSX_U8 v_NoDynTime_u8r, BSX_U8 v_Bandwidth_u8r, BSX_U32 v_oneGthrMax_u32r, BSX_U32 v_oneGthrMin_u32r) {
  s_AccelObserver->V_AccelThreshold_U8R = v_threshold_u8r;
  s_AccelObserver->V_DynamicDetectionBandwidth_U8R = v_Bandwidth_u8r;
  s_AccelObserver->V_NoDynamicDetectionTime_U8R = v_NoDynTime_u8r;
  s_AccelObserver->V_OneGThresMin_U32R = v_oneGthrMin_u32r;
  s_AccelObserver->V_OneGThresMax_U32R = v_oneGthrMax_u32r;
  return 0;
}

BSX_S8 ACCELOBSERVER_Reset(ts_AccelObserver *p_AccelObserver) {
  p_AccelObserver->V_DynamicDetectionCounter_U8R = 0;
  p_AccelObserver->V_DynamicStatus_U8R = 0;
  p_AccelObserver->V_OneGStatus_U8R = 0;
  return 0;
}

BSX_S8 ACCELOBSERVER_Init(ts_AccelObserver *p_accelObserver) {
  BSX_S32 i;
  ACCELOBSERVER_Reset(p_accelObserver);
  p_accelObserver->V_AccelThreshold_U8R = 25;
  p_accelObserver->V_NoDynamicDetectionTime_U8R = 3;
  p_accelObserver->V_OneGThresMax_U32R = 1577536;
  p_accelObserver->V_DynamicDetectionBandwidth_U8R = 4;
  p_accelObserver->V_OneGThresMin_U32R = 553536;
  for (i = 0; i < 5; i++) {
    p_accelObserver->S_AccelVectorBuffer.Ax[i] = 0;
    p_accelObserver->S_AccelVectorBuffer.Ay[i] = 0;
    p_accelObserver->S_AccelVectorBuffer.Az[i] = 0;
  }
  return 0;
}

BSX_S8 ACCELOBSERVER_GetOneGThreshold(ts_AccelObserver *p_AccelObserver, BSX_U32 *v_oneGthrMax_u32r, BSX_U32 *v_oneGthrMin_u32r) {
  *v_oneGthrMax_u32r = p_AccelObserver->V_OneGThresMax_U32R;
  *v_oneGthrMin_u32r = p_AccelObserver->V_OneGThresMin_U32R;
  return 0;
}

BSX_S8 ACCELOBSERVER_GetOneGStatus(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_onegstatus_u8r) {
  *v_onegstatus_u8r = p_AccelObserver->V_OneGStatus_U8R;
  return 0;
}

BSX_S8 ACCELOBSERVER_GetNODynamicDetectionTime(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_NoDynTime_u8r) {
  *v_NoDynTime_u8r = p_AccelObserver->V_NoDynamicDetectionTime_U8R;
  return 0;
}

BSX_S8 ACCELOBSERVER_GetDynamicStatus(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_dynamicstatus_u8r) {
  *v_dynamicstatus_u8r = p_AccelObserver->V_DynamicStatus_U8R;
  return 0;
}

BSX_S8 ACCELOBSERVER_GetDynamicDetectionBandwidth(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_Bandwidth_u8r) {
  *v_Bandwidth_u8r = p_AccelObserver->V_DynamicDetectionBandwidth_U8R;
  return 0;
}

BSX_S8 ACCELOBSERVER_GetAccelDynamicDetectionThreshold(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_threshold_u8r) {
  *v_threshold_u8r = p_AccelObserver->V_AccelThreshold_U8R;
  return 0;
}

BSX_S8 ACCELOBSERVER_DoStep(ts_dataxyzf32 *p_accelVector, ts_AccelObserver *p_accelObserver) {
  ts_dataxyz fifoInXYZ;
  ts_dataxyz fifoRefXYZ;
  BSX_F32 accelVectorLength;
  BSX_S32 ref_index;
  if (p_accelObserver->S_AccelVectorBuffer.insert_index < p_accelObserver->V_DynamicDetectionBandwidth_U8R) {
    ref_index = 5 + p_accelObserver->S_AccelVectorBuffer.insert_index - (BSX_S32)p_accelObserver->V_DynamicDetectionBandwidth_U8R;
  } else {
    ref_index = p_accelObserver->S_AccelVectorBuffer.insert_index - (BSX_S32)p_accelObserver->V_DynamicDetectionBandwidth_U8R;
  }
  fifoInXYZ.x = (BSX_S16)p_accelVector->x;
  fifoInXYZ.y = (BSX_S16)p_accelVector->y;
  fifoInXYZ.z = (BSX_S16)p_accelVector->z;
  MATH_UpdateFIFOBufferXYZ(
    fifoInXYZ.x,
    fifoInXYZ.y,
    fifoInXYZ.z,
    p_accelObserver->S_AccelVectorBuffer.Ax,
    p_accelObserver->S_AccelVectorBuffer.Ay,
    p_accelObserver->S_AccelVectorBuffer.Az,
    &p_accelObserver->S_AccelVectorBuffer.insert_index,
    5);
  fifoRefXYZ.x = p_accelObserver->S_AccelVectorBuffer.Ax[ref_index];
  fifoRefXYZ.y = p_accelObserver->S_AccelVectorBuffer.Ay[ref_index];
  fifoRefXYZ.z = p_accelObserver->S_AccelVectorBuffer.Az[ref_index];
  if (fabsf(fifoInXYZ.x - fifoRefXYZ.x) <= p_accelObserver->V_AccelThreshold_U8R
   && fabsf(fifoInXYZ.y - fifoRefXYZ.y) <= p_accelObserver->V_AccelThreshold_U8R
   && fabsf(fifoInXYZ.z - fifoRefXYZ.z) <= p_accelObserver->V_AccelThreshold_U8R) {
    p_accelObserver->V_DynamicDetectionCounter_U8R++;
    if (p_accelObserver->V_DynamicDetectionCounter_U8R >= 100) {
      p_accelObserver->V_DynamicDetectionCounter_U8R = 100;
    }
  } else {
    p_accelObserver->V_DynamicDetectionCounter_U8R = 0;
  }
  p_accelObserver->V_DynamicStatus_U8R = p_accelObserver->V_DynamicDetectionCounter_U8R < p_accelObserver->V_NoDynamicDetectionTime_U8R;
  accelVectorLength = p_accelVector->x * p_accelVector->x + p_accelVector->y * p_accelVector->y + p_accelVector->z * p_accelVector->z;
  p_accelObserver->V_OneGStatus_U8R = accelVectorLength > (BSX_F32)p_accelObserver->V_OneGThresMin_U32R && accelVectorLength < (BSX_F32)p_accelObserver->V_OneGThresMax_U32R;
  return 0;
}
