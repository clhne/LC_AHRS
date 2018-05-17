#ifndef __MATHOPERATIONS_H__
#define __MATHOPERATIONS_H__
#include "BsxLibraryDataTypes.h"
BSX_F32 Math_CalcVariance(BSX_F32 *Vector_buff, BSX_U16 insert_index, BSX_U16 BuffSize, BSX_U16 StepSize);
BSX_S16 MATH_VectorLength(BSX_S16 x, BSX_S16 y, BSX_S16 z);
BSX_F32 MATH_Variance(BSX_S16 *p_buffer_s16r, BSX_F32 v_Mean_f32r, BSX_U8 v_start, BSX_U8 v_len);
BSX_S16 MATH_UpdateFIFOBufferXYZF32(BSX_F32 dataX, BSX_F32 dataY, BSX_F32 dataZ, BSX_F32 *x_Fifo, BSX_F32 *y_Fifo, BSX_F32 *z_Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength);
BSX_S16 MATH_UpdateFIFOBufferXYZ(BSX_S16 dataX, BSX_S16 dataY, BSX_S16 dataZ, BSX_S16 *x_Fifo, BSX_S16 *y_Fifo, BSX_S16 *z_Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength);
BSX_S16 MATH_UpdateFIFOBufferF32(BSX_F32 data, BSX_F32 *Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength);
BSX_S16 MATH_UpdateFIFOBuffer(BSX_S16 data, BSX_S16 *Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength);
BSX_F32 MATH_StandardDeviation(BSX_S16 *p_buffer_s16r, BSX_U8 v_start, BSX_U8 v_len);
BSX_F32 MATH_Round(BSX_F32 x);
BSX_F32 MATH_MeanFL(BSX_F32 *p_buffer_f32r, BSX_S16 v_start, BSX_S16 v_len);
BSX_F32 MATH_Mean(BSX_S16 *p_buffer_s16r, BSX_S16 v_start, BSX_S16 v_len);
BSX_S16 MATH_GetMinMax(BSX_S16 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len, BSX_S16 *minVal, BSX_S16 *maxVal);
BSX_S16 MATH_GetMin(BSX_S16 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len, BSX_S16 *minVal);
BSX_F32 MATH_GetMaxFL(BSX_F32 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len);
BSX_S16 MATH_GetMax(BSX_S16 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len, BSX_S16 *maxVal);
#endif