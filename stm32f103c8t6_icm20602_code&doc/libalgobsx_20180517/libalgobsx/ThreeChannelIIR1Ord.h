#ifndef __BSX_THREECHANNELIIR1ORD_H__
#define __BSX_THREECHANNELIIR1ORD_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_F32 V_iirCoef_F32R;
  ts_dataxyzf32 S_filtData;
} ts_iirFilter;
BSX_S8 THREECHANNELIIR1ORD_SetCoefficient(ts_iirFilter *p_iirFilter, BSX_F32 v_Value_f32r);
BSX_S8 THREECHANNELIIR1ORD_Run(ts_iirFilter *p_iirFilter, float a2, float a3, float a4, ts_dataxyzf32 s_rawData, ts_dataxyzf32 *p_filtData);
BSX_S8 THREECHANNELIIR1ORD_Reset(ts_iirFilter *p_iirFilter);
BSX_S8 THREECHANNELIIR1ORD_Init(ts_iirFilter *p_iirFilter);
#endif