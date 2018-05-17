#ifndef __BSX_OBSERVERGYRO_H__
#define __BSX_OBSERVERGYRO_H__
#include "BsxLibraryDataTypes.h"
typedef struct{
  BSX_U16 iteration;
  BSX_S16 Wx_buf[25];
  BSX_S16 Wy_buf[25];
  BSX_S16 Wz_buf[25];
  BSX_U8 insert_index;
} ts_GYRO_RATES_BUFFER;
typedef struct{
  ts_GYRO_RATES_BUFFER S_GyroRatesBuffer;
  BSX_S16 MinValX;
  BSX_S16 MinValY;
  BSX_S16 MinValZ;
  BSX_S16 MaxValX;
  BSX_S16 MaxValY;
  BSX_S16 MaxValZ;
  BSX_U16 V_ShakeDetectThresh_U16R;
  BSX_U8 V_DynamicStatus_U8R;
  BSX_U8 V_DynDetCounter_U8R;
  BSX_U8 V_NoDynDetTime_U8R;
  BSX_U8 V_FifoSize_U8R;
  BSX_U8 V_DynDetThresh_U8R;
  BSX_S8 V_ShakeDetectTime_S8R;
  BSX_S8 V_ShakeDetectCounter_S8R;
  BSX_U8 V_ShakeStatus_U8R;
} ts_GYROBSERVER;
BSX_U8 GYROOBSERVER_SetShakeDetectionParam(ts_GYROBSERVER *p_GyroObserver, BSX_U16 v_detectThreshold_u16r, BSX_S8 v_detectionTime_s8r);
BSX_S8 GYROOBSERVER_SetNoDynamicDetectionTime(ts_GYROBSERVER *p_GyroObserver, BSX_U8 v_noDynDetTime_u8r);
BSX_S8 GYROOBSERVER_SetDynamicDetectionThreshold(ts_GYROBSERVER *p_GyroObserver, BSX_U8 v_gyrodynthres_u8r);
BSX_U8 GYROOBSERVER_Reset(ts_GYROBSERVER *p_GyroObserver);
BSX_U8 GYROOBSERVER_Init(ts_GYROBSERVER *p_GyroObserver, BSX_U8 v_gyrohwid_u8);
BSX_U8 GYROOBSERVER_GetShakeStatus(ts_GYROBSERVER *p_GyroObserver, BSX_U8 *v_status_u8r);
BSX_S8 GYROOBSERVER_GetNoDynamicDetectionTime(ts_GYROBSERVER *p_GyroObserver, BSX_U8 *v_noDynDetTime_u8r);
BSX_S8 GYROOBSERVER_GetDynamicDetectionThreshold(ts_GYROBSERVER *p_GyroObserver, BSX_U8 *v_gyrodynthres_u8r);
BSX_U8 GYROOBSERVER_DynamicDetection(ts_dataxyzf32 *Gyro_Data_Raw, ts_GYROBSERVER *p_GyroObserver);
BSX_U8 GYROOBSERVER_DoStep(ts_dataxyzf32 *p_gyrodataraw, ts_GYROBSERVER *p_GyroObserver);
#endif