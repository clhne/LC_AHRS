#include "ThreeChannelIIR1Ord.h"

BSX_S8 THREECHANNELIIR1ORD_SetCoefficient(ts_iirFilter *p_iirFilter, BSX_F32 v_Value_f32r) {
  p_iirFilter->V_iirCoef_F32R = v_Value_f32r;
  return 0;
}

BSX_S8 THREECHANNELIIR1ORD_Run(ts_iirFilter *p_iirFilter, float a2, float a3, float a4, ts_dataxyzf32 s_rawData, ts_dataxyzf32 *p_filtData) {
  BSX_F32 prev_coef = p_iirFilter->V_iirCoef_F32R;
  BSX_F32 cur_coef = 1.0f - prev_coef;
  p_iirFilter->S_filtData.x = cur_coef * s_rawData.x + prev_coef * p_iirFilter->S_filtData.x;
  p_iirFilter->S_filtData.y = cur_coef * s_rawData.y + prev_coef * p_iirFilter->S_filtData.y;
  p_iirFilter->S_filtData.z = cur_coef * s_rawData.z + prev_coef * p_iirFilter->S_filtData.z;
  p_filtData->x = p_iirFilter->S_filtData.x;
  p_filtData->y = p_iirFilter->S_filtData.y;
  p_filtData->z = p_iirFilter->S_filtData.z;
  return 0;
}

BSX_S8 THREECHANNELIIR1ORD_Reset(ts_iirFilter *p_iirFilter) {
  p_iirFilter->S_filtData.x = 0.0;
  p_iirFilter->S_filtData.y = 0.0;
  p_iirFilter->S_filtData.z = 0.0;
  return 0;
}

BSX_S8 THREECHANNELIIR1ORD_Init(ts_iirFilter *p_iirFilter) {
  THREECHANNELIIR1ORD_SetCoefficient(p_iirFilter, 0.5f);
  THREECHANNELIIR1ORD_Reset(p_iirFilter);
  return 0;
}
