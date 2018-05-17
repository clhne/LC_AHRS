#include <math.h>
#include "MathOperations.h"
#include "MatrixOperations.h"
#include "EKFbasedCalibration.h"

BSX_S8 EKF_SetProcessNoise(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_ProcessNoise_f32r) {
  MATRIX_Copy(a_ProcessNoise_f32r, p_ekfCalib->A_processNoise_F32R, 1, 4);
  return 0;
}

BSX_S8 EKF_SetMeasureNoise(ts_ekfCalib *p_ekfCalib, BSX_F32 v_MeasureNoise_f32r) {
  p_ekfCalib->V_measureNoise_F32R = v_MeasureNoise_f32r;
  return 0;
}

BSX_S8 EKF_SetCovariance(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_Covariance_f32r) {
  MATRIX_Diagonal(a_Covariance_f32r, p_ekfCalib->A_covariance_F32R, 1, 4);
  return 0;
}

BSX_S8 EKF_SensorErrorModel(ts_dataxyz s_InXYZ, ts_dataxyzf32 s_OffXYZ, ts_dataxyzf32 s_NormXYZ, ts_dataxyzf32 *p_CorrXYZ) {
  p_CorrXYZ->x = ((BSX_F32)s_InXYZ.x - s_OffXYZ.x) + s_NormXYZ.x;
  p_CorrXYZ->y = ((BSX_F32)s_InXYZ.y - s_OffXYZ.y) + s_NormXYZ.y;
  p_CorrXYZ->z = ((BSX_F32)s_InXYZ.z - s_OffXYZ.z) + s_NormXYZ.z;
  return 0;
}

BSX_S8 EKF_Run(ts_ekfCalib *p_ekfCalib, ts_dataxyz s_XYZData) {
  BSX_F32 v_Innovation_F32R;
  BSX_F32 v_Temp_F32R;
  ts_dataxyzf32 s_OffXYZ;
  ts_dataxyzf32 s_NormXYZ;
  ts_dataxyzf32 s_CorXYZ;
  BSX_F32 a_KalmanGain_F32R[4];
  BSX_F32 a_MeasMatrix_F32R[16];
  BSX_F32 a_ExpCovar_F32R[16];
  BSX_F32 a_TempMatrixA_F32R[16];
  BSX_F32 a_TempMatrixB_F32R[16];
  s_NormXYZ.x = 0.0;
  s_NormXYZ.y = 0.0;
  s_NormXYZ.z = 0.0;
  s_OffXYZ.x = p_ekfCalib->A_sysParam_F32R[0];
  s_OffXYZ.y = p_ekfCalib->A_sysParam_F32R[1];
  s_OffXYZ.z = p_ekfCalib->A_sysParam_F32R[2];
  EKF_SensorErrorModel(s_XYZData, s_OffXYZ, s_NormXYZ, &s_CorXYZ);
  MATRIX_Diagonal(p_ekfCalib->A_processNoise_F32R, a_MeasMatrix_F32R, 1, 4);
  MATRIX_Add(p_ekfCalib->A_covariance_F32R, a_MeasMatrix_F32R, a_ExpCovar_F32R, 4, 4);
  a_MeasMatrix_F32R[0] = s_CorXYZ.x * -2.0f;
  a_MeasMatrix_F32R[1] = s_CorXYZ.y * -2.0f;
  a_MeasMatrix_F32R[2] = s_CorXYZ.z * -2.0f;
  a_MeasMatrix_F32R[3] = p_ekfCalib->A_sysParam_F32R[3] * -2.0f;
  MATRIX_Multiply(a_ExpCovar_F32R, a_MeasMatrix_F32R, a_TempMatrixA_F32R, 4, 4, 1);
  MATRIX_Multiply(a_MeasMatrix_F32R, a_ExpCovar_F32R, a_KalmanGain_F32R, 1, 4, 4);
  MATRIX_Multiply(a_KalmanGain_F32R, a_MeasMatrix_F32R, &v_Temp_F32R, 1, 4, 1);
  v_Temp_F32R = 1.0f / (v_Temp_F32R + p_ekfCalib->V_measureNoise_F32R);
  MATRIX_Multiply(a_TempMatrixA_F32R, &v_Temp_F32R, a_KalmanGain_F32R, 4, 1, 1);
  v_Innovation_F32R = EKF_ObservationModel(0, s_CorXYZ, p_ekfCalib->A_sysParam_F32R[3]);
  MATRIX_Identity(a_TempMatrixA_F32R, 4);
  MATRIX_Multiply(a_KalmanGain_F32R, a_MeasMatrix_F32R, a_TempMatrixB_F32R, 4, 1, 4);
  MATRIX_Subtract(a_TempMatrixA_F32R, a_TempMatrixB_F32R, a_TempMatrixA_F32R, 4, 4);
  MATRIX_Multiply(a_TempMatrixA_F32R, a_ExpCovar_F32R, a_MeasMatrix_F32R, 4, 4, 4);
  MATRIX_Multiply(a_KalmanGain_F32R, &v_Innovation_F32R, a_TempMatrixB_F32R, 4, 1, 1);
  MATRIX_Add(a_TempMatrixB_F32R, p_ekfCalib->A_sysParam_F32R, a_KalmanGain_F32R, 4, 1);
  if (a_KalmanGain_F32R[3] > (BSX_F32)p_ekfCalib->V_RThresMax_S16R) {
    a_KalmanGain_F32R[3] = (BSX_F32)p_ekfCalib->V_RThresMax_S16R;
  }
  if (a_KalmanGain_F32R[3] < (BSX_F32)p_ekfCalib->V_RThresMin_S16R) {
    a_KalmanGain_F32R[3] = (BSX_F32)p_ekfCalib->V_RThresMin_S16R;
  }
  MATRIX_Copy(a_KalmanGain_F32R, p_ekfCalib->A_sysParam_F32R, 1, 4);
  MATRIX_Copy(a_MeasMatrix_F32R, p_ekfCalib->A_covariance_F32R, 4, 4);
  return 0;
}

BSX_S8 EKF_ObservationModel_SphereCosPhi(ts_dataxyzf32 s_CorrXYZ, BSX_F32 v_Radius_f32r, BSX_F32 *p_Innovation) {
  BSX_F32 length = sqrtf(s_CorrXYZ.y * s_CorrXYZ.y + s_CorrXYZ.x * s_CorrXYZ.x + s_CorrXYZ.z * s_CorrXYZ.z);
  p_Innovation[0] = s_CorrXYZ.x - s_CorrXYZ.x * length * 1.0f / v_Radius_f32r;
  p_Innovation[1] = s_CorrXYZ.y - s_CorrXYZ.y * length * 1.0f / v_Radius_f32r;
  p_Innovation[2] = s_CorrXYZ.z - s_CorrXYZ.z * length * 1.0f / v_Radius_f32r;
  p_Innovation[3] = v_Radius_f32r * v_Radius_f32r - length * length;
  return 0;
}

BSX_F32 EKF_ObservationModel(BSX_S32 meas, ts_dataxyzf32 s_CorXYZ, BSX_F32 v_Radius_f32r) {
  return (BSX_F32)meas - s_CorXYZ.x * s_CorXYZ.x - s_CorXYZ.y * s_CorXYZ.y - s_CorXYZ.z * s_CorXYZ.z + v_Radius_f32r * v_Radius_f32r;
}

BSX_S8 EKF_InitSysParam(BSX_F32 *p_SysParam_f32r, ts_dataxyzf32 s_Offset, BSX_F32 v_Radius_f32r) {
  p_SysParam_f32r[0] = s_Offset.x;
  p_SysParam_f32r[1] = s_Offset.y;
  p_SysParam_f32r[2] = s_Offset.z;
  p_SysParam_f32r[3] = v_Radius_f32r;
  return 0;
}

BSX_S8 EKF_Init(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_Covariance_f32r, BSX_F32 *a_ProcessNoise_f32r, BSX_F32 v_MeasureNoise_f32r) {
  EKF_SetCovariance(p_ekfCalib, a_Covariance_f32r);
  EKF_SetProcessNoise(p_ekfCalib, a_ProcessNoise_f32r);
  EKF_SetMeasureNoise(p_ekfCalib, v_MeasureNoise_f32r);
  p_ekfCalib->V_RThresMax_S16R = 300;
  p_ekfCalib->A_sysParam_F32R[0] = 1.0;
  p_ekfCalib->A_sysParam_F32R[1] = 1.0;
  p_ekfCalib->A_sysParam_F32R[2] = 1.0;
  p_ekfCalib->A_sysParam_F32R[3] = 1.0;
  p_ekfCalib->V_RThresMin_S16R = 30;
  p_ekfCalib->V_InnovThres_S16R = 45;
  return 0;
}

BSX_S8 EKF_GetSphereParam(ts_ekfCalib *p_ekfCalib, ts_dataxyz *p_DataXYZ, BSX_S16 *p_Radius) {
  p_DataXYZ->x = (BSX_S16)MATH_Round(p_ekfCalib->A_sysParam_F32R[0]);
  p_DataXYZ->y = (BSX_S16)MATH_Round(p_ekfCalib->A_sysParam_F32R[1]);
  p_DataXYZ->z = (BSX_S16)MATH_Round(p_ekfCalib->A_sysParam_F32R[2]);
  *p_Radius = (BSX_S16)MATH_Round(p_ekfCalib->A_sysParam_F32R[3]);
  return 0;
}

BSX_S8 EKF_GetProcessNoise(ts_ekfCalib *p_ekfCalib, BSX_F32 *a_ProcessNoise_f32r) {
  MATRIX_Copy(p_ekfCalib->A_processNoise_F32R, a_ProcessNoise_f32r, 1, 4);
  return 0;
}
