#include <math.h>
#include "MathOperations.h"
BSX_F32 Math_CalcVariance(BSX_F32 *Vector_buff, BSX_U16 insert_index, BSX_U16 BuffSize, BSX_U16 StepSize) {
  BSX_S32 i;
  BSX_S16 last_index;
  BSX_S32 from_index;
  BSX_F32 mean;
  BSX_F32 var;
  BSX_F32 diff;
  if (insert_index) {
    last_index = insert_index - 1;
  } else {
    last_index = BuffSize - 1;
  }
  if (last_index >= StepSize - 1) {
    from_index = 1 - StepSize + last_index;
  } else {
    from_index = BuffSize + 1 - StepSize + last_index;
  }
  if (!from_index) {
    from_index = BuffSize;
  }
  mean = 0.0f;
  for (i = 0; i <= (BSX_S32)last_index; i++) {
    mean += Vector_buff[i];
  }
  for (i = from_index; i < (BSX_S32)BuffSize; i++) {
    mean += Vector_buff[i];
  }
  if (StepSize) {
    mean = mean / (BSX_F32)StepSize;
  }
  var = 0.0f;
  for (i = 0; i <= (BSX_S32)last_index; i++) {
    diff = Vector_buff[i] - mean;
    var += diff * diff;
  }
  for (i = from_index; i < (BSX_S32)BuffSize; i++) {
    diff = Vector_buff[i] - mean;
    var += diff * diff;
  }
  if (StepSize != 1) {
    var = var / (BSX_F32)(StepSize - 1);
  }
  return var;
}

BSX_S16 MATH_VectorLength(BSX_S16 x, BSX_S16 y, BSX_S16 z) {
  BSX_F32 norm = sqrtf((BSX_F32)(y * y + x * x + z * z));;
  if (norm > 32767.0) {
    return (BSX_S16)0x7FFF;
  } else {
    return (BSX_S16)norm;
  }
}

BSX_F32 MATH_Variance(BSX_S16 *p_buffer_s16r, BSX_F32 v_Mean_f32r, BSX_U8 v_start, BSX_U8 v_len) {
  BSX_S32 i;
  BSX_F32 diff = p_buffer_s16r[v_start] - v_Mean_f32r;
  BSX_F32 sum = diff * diff;
  for (i = 1; i < v_len; i++) {
    diff = p_buffer_s16r[v_start + i] - v_Mean_f32r;
    sum += diff * diff;
  }
  return sum / (BSX_F32)(v_len - 1);
}

BSX_S16 MATH_UpdateFIFOBufferXYZF32(BSX_F32 dataX, BSX_F32 dataY, BSX_F32 dataZ, BSX_F32 *x_Fifo, BSX_F32 *y_Fifo, BSX_F32 *z_Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength) {
  BSX_S32 idx = *v_Index_u8r;
  x_Fifo[idx] = dataX;
  y_Fifo[idx] = dataY;
  z_Fifo[idx] = dataZ;
  *v_Index_u8r = (BSX_U32)(idx + 1) % v_BuffLength;
  return 0;
}

BSX_S16 MATH_UpdateFIFOBufferXYZ(BSX_S16 dataX, BSX_S16 dataY, BSX_S16 dataZ, BSX_S16 *x_Fifo, BSX_S16 *y_Fifo, BSX_S16 *z_Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength) {
  BSX_S32 idx = *v_Index_u8r;
  x_Fifo[idx] = dataX;
  y_Fifo[idx] = dataY;
  z_Fifo[idx] = dataZ;
  *v_Index_u8r = (BSX_U32)(idx + 1) % v_BuffLength;
  return 0;
}

BSX_S16 MATH_UpdateFIFOBufferF32(BSX_F32 data, BSX_F32 *Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength) {
  BSX_S32 idx = *v_Index_u8r;
  Fifo[idx] = data;
  *v_Index_u8r = (BSX_U32)(idx + 1) % v_BuffLength;
  return 0;
}

BSX_S16 MATH_UpdateFIFOBuffer(BSX_S16 data, BSX_S16 *Fifo, BSX_U8 *v_Index_u8r, BSX_U16 v_BuffLength) {
  BSX_S32 idx = *v_Index_u8r;
  Fifo[idx] = data;
  *v_Index_u8r = (BSX_U32)(idx + 1) % v_BuffLength;
  return 0;
}

BSX_F32 MATH_StandardDeviation(BSX_S16 *p_buffer_s16r, BSX_U8 v_start, BSX_U8 v_len) {
  BSX_F32 mean = MATH_Mean(p_buffer_s16r, v_start, v_len);
  BSX_F32 var = MATH_Variance(p_buffer_s16r, mean, v_start, v_len);
  return sqrtf(var);
}

BSX_F32 MATH_Round(BSX_F32 x) {
  BSX_F32 result;
  if (x < 0.0) {
    result = ceilf(x - 0.5);
  } else {
    result = floorf(x + 0.5);
  }
  return result;
}

BSX_F32 MATH_MeanFL(BSX_F32 *p_buffer_f32r, BSX_S16 v_start, BSX_S16 v_len) {
  BSX_S32 i;
  BSX_F32 sum = p_buffer_f32r[v_start];
  for (i = 1; i < v_len; i++) {
    BSX_F32 val = p_buffer_f32r[v_start + i];
    sum += val;
  }
  return sum / (BSX_F32)v_len;
}

BSX_F32 MATH_Mean(BSX_S16 *p_buffer_s16r, BSX_S16 v_start, BSX_S16 v_len) {
  BSX_S32 i;
  BSX_F32 sum = (BSX_F32)p_buffer_s16r[v_start];
  for (i = 1; i < v_len; i++) {
    BSX_S16 val = p_buffer_s16r[v_start + i];
    sum += (BSX_F32)val;
  }
  return sum / (BSX_F32)v_len;
}

BSX_S16 MATH_GetMinMax(BSX_S16 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len, BSX_S16 *minVal, BSX_S16 *maxVal) {
  BSX_S32 i;
  BSX_S16 min = a_Fifo[v_start];
  BSX_S16 max = min;
  for (i = 1; i < v_len; i++) {
    BSX_S16 val = a_Fifo[v_start + i];
    if (val >= max) {
      max = val;
    }
    if (val < min) {
      min = val;
    }
  }
  *maxVal = max;
  *minVal = min;
  return 0;
}

BSX_S16 MATH_GetMin(BSX_S16 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len, BSX_S16 *minVal) {
  BSX_S32 i;
  BSX_S16 min = a_Fifo[v_start];
  for (i = 1; i < v_len; i++) {
    BSX_S16 val = a_Fifo[v_start + i];
    if (val < min) {
      min = val;
    }
  }
  *minVal = min;
  return 0;
}

BSX_F32 MATH_GetMaxFL(BSX_F32 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len) {
  BSX_S32 i;
  BSX_F32 max = a_Fifo[v_start];
  for (i = 1; i < v_len; i++) {
    BSX_F32 val = a_Fifo[v_start + i];
    if (val > max) {
      max = val;
    }
  }
  return max;
}

BSX_S16 MATH_GetMax(BSX_S16 *a_Fifo, BSX_S16 v_start, BSX_S16 v_len, BSX_S16 *maxVal) {
  BSX_S32 i;
  BSX_S16 max = a_Fifo[v_start];
  for (i = 1; i < v_len; i++) {
    BSX_S16 val = a_Fifo[v_start + i];
    if (val >= max) {
      max = val;
    }
  }
  *maxVal = max;
  return 0;
}
