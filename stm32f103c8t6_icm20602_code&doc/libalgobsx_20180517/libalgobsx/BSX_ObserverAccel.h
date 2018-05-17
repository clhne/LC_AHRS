#ifndef __BSX_OBSERVERACCEL_H__
#define __BSX_OBSERVERACCEL_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_S16 Ax[5];
  BSX_S16 Ay[5];
  BSX_S16 Az[5];
  BSX_U8 insert_index;
} ts_ACCEL_VECTOR_BUFF;
typedef struct {
  BSX_U32 V_OneGThresMax_U32R;
  BSX_U32 V_OneGThresMin_U32R;
  BSX_U8 V_DynamicStatus_U8R;
  BSX_U8 V_OneGStatus_U8R;
  BSX_U8 V_BufferLength_U8R;
  BSX_U8 V_DynamicDetectionCounter_U8R;
  BSX_U8 V_NoDynamicDetectionTime_U8R;
  BSX_U8 V_DynamicDetectionBandwidth_U8R;
  BSX_U8 V_AccelThreshold_U8R;
  ts_ACCEL_VECTOR_BUFF S_AccelVectorBuffer;
} ts_AccelObserver;
BSX_S8 ACCELOBSERVER_SetParameterSettings(ts_AccelObserver *s_AccelObserver, BSX_U8 v_threshold_u8r, BSX_U8 v_NoDynTime_u8r, BSX_U8 v_Bandwidth_u8r, BSX_U32 v_oneGthrMax_u32r, BSX_U32 v_oneGthrMin_u32r);
BSX_S8 ACCELOBSERVER_Reset(ts_AccelObserver *p_AccelObserver);
BSX_S8 ACCELOBSERVER_Init(ts_AccelObserver *p_accelObserver);
BSX_S8 ACCELOBSERVER_GetOneGThreshold(ts_AccelObserver *p_AccelObserver, BSX_U32 *v_oneGthrMax_u32r, BSX_U32 *v_oneGthrMin_u32r);
BSX_S8 ACCELOBSERVER_GetOneGStatus(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_onegstatus_u8r);
BSX_S8 ACCELOBSERVER_GetNODynamicDetectionTime(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_NoDynTime_u8r);
BSX_S8 ACCELOBSERVER_GetDynamicStatus(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_dynamicstatus_u8r);
BSX_S8 ACCELOBSERVER_GetDynamicDetectionBandwidth(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_Bandwidth_u8r);
BSX_S8 ACCELOBSERVER_GetAccelDynamicDetectionThreshold(ts_AccelObserver *p_AccelObserver, BSX_U8 *v_threshold_u8r);
BSX_S8 ACCELOBSERVER_DoStep(ts_dataxyzf32 *p_accelVector, ts_AccelObserver *p_accelObserver);
#endif