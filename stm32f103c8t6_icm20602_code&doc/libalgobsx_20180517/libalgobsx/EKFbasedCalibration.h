#ifndef __EKFBASEDCALIBRATION_H__
#define __EKFBASEDCALIBRATION_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_F32 A_sysParam_F32R[4];
  BSX_F32 A_covariance_F32R[16];
  BSX_F32 A_processNoise_F32R[4];
  BSX_F32 V_measureNoise_F32R;
  BSX_S16 V_RThresMin_S16R;
  BSX_S16 V_RThresMax_S16R;
  BSX_S16 V_InnovThres_S16R;
} ts_ekfCalib;
BSX_S8 EKF_SetProcessNoise(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_ProcessNoise_f32r);
BSX_S8 EKF_SetMeasureNoise(ts_ekfCalib *p_ekfCalib, BSX_F32 v_MeasureNoise_f32r);
BSX_S8 EKF_SetCovariance(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_Covariance_f32r);
BSX_S8 EKF_SensorErrorModel(ts_dataxyz s_InXYZ, ts_dataxyzf32 s_OffXYZ, ts_dataxyzf32 s_NormXYZ, ts_dataxyzf32 *p_CorrXYZ);
BSX_S8 EKF_Run(ts_ekfCalib *p_ekfCalib, ts_dataxyz s_XYZData);
BSX_S8 EKF_ObservationModel_SphereCosPhi(ts_dataxyzf32 s_CorrXYZ, BSX_F32 v_Radius_f32r, BSX_F32 *p_Innovation);
BSX_F32 EKF_ObservationModel(BSX_S32 meas, ts_dataxyzf32 s_CorXYZ, BSX_F32 v_Radius_f32r);
BSX_S8 EKF_InitSysParam(BSX_F32 *p_SysParam_f32r, ts_dataxyzf32 s_Offset, BSX_F32 v_Radius_f32r);
BSX_S8 EKF_Init(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_Covariance_f32r, BSX_F32 *a_ProcessNoise_f32r, BSX_F32 v_MeasureNoise_f32r);
BSX_S8 EKF_GetSphereParam(ts_ekfCalib *p_ekfCalib, ts_dataxyz *p_DataXYZ, BSX_S16 *p_Radius);
BSX_S8 EKF_GetProcessNoise(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_ProcessNoise_f32r);
#endif