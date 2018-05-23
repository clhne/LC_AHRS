#include <math.h>
#include "MatrixOperations.h"
BSX_S8 MATRIX_Transpose(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i, j;
  for (i = 0; i < (BSX_S32)v_Rows_u8r; i++) {
    for (j = 0; j < (BSX_S32)v_Columns_u8r; j++) {
      p_OutMatrix_f32r[j * v_Rows_u8r + i] = p_InMatrix_f32r[j];
    }
    p_InMatrix_f32r += v_Columns_u8r;
  }
  return 0;
}

BSX_S8 MATRIX_Subtract(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrixA_f32r[i] - p_InMatrixB_f32r[i];
  }
  return 0;
}

BSX_S8 MATRIX_Multiply_s32(BSX_S32 *p_InMatrixA_s32r, BSX_S32 *p_InMatrixB_s32r, BSX_S32 *p_OutMatrix_s32r, BSX_U8 v_RowsA_u8r, BSX_U8 v_RowsB_u8r, BSX_U8 v_ColumnsB_u8r) {
  BSX_S32 i, j, k;
  for (i = 0; i < (BSX_S32)v_RowsA_u8r; i++) {
    for (j = 0; j < (BSX_S32)v_ColumnsB_u8r; j++) {
      BSX_S32 sum = 0;
      for (k = 0; k < (BSX_S32)v_RowsB_u8r; k++) {
        sum += p_InMatrixA_s32r[k] * p_InMatrixB_s32r[k * v_ColumnsB_u8r + j];
      }
      p_OutMatrix_s32r[j] = sum;
    }
    p_OutMatrix_s32r += v_ColumnsB_u8r;
    p_InMatrixA_s32r += v_RowsB_u8r;
  }
  return 0;
}

BSX_S8 MATRIX_Multiply(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_RowsA_u8r, BSX_U8 v_RowsB_u8r, BSX_U8 v_ColumnsB_u8r) {
  BSX_S32 i, j, k;
  for (i = 0; i < (BSX_S32)v_RowsA_u8r; i++) {
    for (j = 0; j < (BSX_S32)v_ColumnsB_u8r; j++) {
      BSX_F32 sum = 0.0f;
      for (k = 0; k < (BSX_S32)v_RowsB_u8r; k++) {
        sum += p_InMatrixA_f32r[k] * p_InMatrixB_f32r[k * v_ColumnsB_u8r + j];
      }
      p_OutMatrix_f32r[j] = sum;
    }
    p_OutMatrix_f32r += v_ColumnsB_u8r;
    p_InMatrixA_f32r += v_RowsB_u8r;
  }
  return 0;
}

BSX_S8 MATRIX_MeanFixed(BSX_S16 *p_InMatrix_s16r, BSX_S16 *p_OutMatrix_s16r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r) {
  BSX_S32 i, j;
  for (i = 0; i < (BSX_S32)v_row_u8r; i++) {
    BSX_F32 sum = 0.0f;
    for (j = 0; j < (BSX_S32)v_column_u8r; j++) {
      sum += (BSX_F32)p_InMatrix_s16r[j];
    }
    p_InMatrix_s16r += v_column_u8r;
    *p_OutMatrix_s16r = (BSX_S32)(sum / (BSX_F32)v_column_u8r);
    ++p_OutMatrix_s16r;
  }
  return 0;
}

BSX_S8 MATRIX_Mean(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r) {
  BSX_S32 i, j;
  for (i = 0; i < (BSX_S32)v_row_u8r; i++) {
    BSX_F32 sum = 0.0f;
    for (j = 0; j < (BSX_S32)v_column_u8r; j++) {
      sum += p_InMatrix_f32r[j];
    }
    p_InMatrix_f32r += v_column_u8r;
    *p_OutMatrix_f32r = sum / (BSX_F32)v_column_u8r;
    ++p_OutMatrix_f32r;;
  }
  return 0;
}

BSX_S8 MATRIX_Inverse(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_dim_u8r, BSX_U8 v_LSEMethod_u8r) { // v_dim_u8r <= 4
  BSX_S32 i, j;
  BSX_F32 a_EyeMatrix_f32r[4];
  BSX_F32 a_ColMatrix_f32r[4];
  BSX_F32 a_MatrixA_f32r[4][4];
  if (v_LSEMethod_u8r != 1) {
    return 0;
  }
  for (i = 0; i < (BSX_S32)v_dim_u8r; i++) {
    for (j = 0; j < (BSX_S32)v_dim_u8r; j++) {
      a_EyeMatrix_f32r[j] = 0.0f;
    }
    a_EyeMatrix_f32r[i] = 1.0f;
    MATRIX_Copy(p_InMatrix_f32r, (BSX_F32 *)a_MatrixA_f32r, 4, 4);
    MATRIX_GaussElimination(v_dim_u8r, (BSX_F32 *)a_MatrixA_f32r, (BSX_F32 *)a_EyeMatrix_f32r, (BSX_F32 *)a_ColMatrix_f32r);
    for (j = 0; j < (BSX_S32)v_dim_u8r; j++) {
      p_OutMatrix_f32r[i + j * (BSX_S32)v_dim_u8r] = a_ColMatrix_f32r[j];
    }
  }
  return 0;
}

BSX_S8 MATRIX_Initialize_s32(BSX_S32 *p_InMatrix_s32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r, BSX_S32 v_initValue_s16r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_column_u8r * (BSX_S32)v_row_u8r;
  for (i = 0; i < size; i++) {
    p_InMatrix_s32r[i] = (BSX_S32)v_initValue_s16r;
  }
  return 0;
}

BSX_S8 MATRIX_InitializeFixed(BSX_S16 *p_InMatrix_s16r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r, BSX_S16 v_initValue_s16r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_column_u8r * (BSX_S32)v_row_u8r;
  for (i = 0; i < size; i++) {
    p_InMatrix_s16r[i] = v_initValue_s16r;
  }
  return 0;
}

BSX_S8 MATRIX_Initialize(BSX_F32 *p_InMatrix_f32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r, BSX_F32 v_initValue_f32r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_column_u8r * (BSX_S32)v_row_u8r;
  for (i = 0; i < size; i++) {
    p_InMatrix_f32r[i] = v_initValue_f32r;
  }
  return 0;
}

BSX_S8 MATRIX_Identity(BSX_F32 *p_InMatrix_f32r, BSX_U8 v_dim_u8r) {
  BSX_S32 i, j;
  for (i = 0; i < (BSX_S32)v_dim_u8r; i++) {
    for (j = 0; j < (BSX_S32)v_dim_u8r; j++) {
      if (j == i) {
        p_InMatrix_f32r[j] = 1.0f;
      } else {
        p_InMatrix_f32r[j] = 0.0f;
      }
    }
    p_InMatrix_f32r += v_dim_u8r;
  }
  return 0;
}

BSX_S8 MATRIX_GaussJordanElimination(BSX_U8 v_rows, BSX_U8 v_colA, BSX_U8 v_colB, BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r) {
  // only for v_rows=3, v_colA=3, v_colB=6, p_InMatrixA_f32r is 3x3 matrix, p_InMatrixB_f32r is 3x3 matrix and p_OutMatrix_f32r is 3x6 matrix
  BSX_S32 i, j, k;
  BSX_F32 augmentedmatrix[3][9];
  for (i = 0; i < (BSX_S32)v_rows; i++) {
    for (j = 0; j < (BSX_S32)v_colA; j++) {
      augmentedmatrix[i][j] = p_InMatrixA_f32r[j + i * v_colA];
    }
    for (j = 0; j < (BSX_S32)v_colB; j++) {
      augmentedmatrix[i][j + v_colA] = p_InMatrixB_f32r[j + i * v_colB];
    }
  }
  for (i = 0; i < (BSX_S32)v_rows; i++) {
    for (j = 0; j < (BSX_S32)v_rows; j++) {
      if (i != j) {
        BSX_F32 scale_factor = augmentedmatrix[j][i] / augmentedmatrix[i][i];
        for (k = 0; k < (BSX_S32)v_colA + (BSX_S32)v_colB; k++) {
          augmentedmatrix[j][k] -= scale_factor * augmentedmatrix[i][k];
        }
      }
    }
  }
  for (i = 0; i < (BSX_S32)v_rows; i++) {
    BSX_F32 scale_factor = augmentedmatrix[i][i];
    for (j = 0; j < 6; j++) {
      p_OutMatrix_f32r[j + 6 * i] = augmentedmatrix[i][v_rows + j] / scale_factor;
    }
  }
  return 0;
}

BSX_S8 MATRIX_GaussElimination(BSX_S32 v_nDim_BSX_S32r, BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r) {
  BSX_S32 i, j, k;
  BSX_F32 swap;
  for (i = 0; i < v_nDim_BSX_S32r; i++) {
    // find pivot element index
    BSX_F32 pivot_element_value = p_InMatrixA_f32r[i * v_nDim_BSX_S32r + i];
    BSX_F32 pivot_element_dist = fabsf(pivot_element_value);
    BSX_S32 pivot_element_row = i;
    for (j = i + 1; j < v_nDim_BSX_S32r; j++) {
      BSX_F32 value = p_InMatrixA_f32r[j * v_nDim_BSX_S32r + i];
      BSX_F32 dist = fabsf(value);
      if (pivot_element_dist < dist) {
        pivot_element_value = value;
        pivot_element_dist = dist;
        pivot_element_row = j;
      }
    }
    if (pivot_element_row != i) {
      // swap pivot element row with current row i
      for (j = i; j < v_nDim_BSX_S32r; j++) {
        swap = p_InMatrixA_f32r[i * v_nDim_BSX_S32r + j];
        p_InMatrixA_f32r[i * v_nDim_BSX_S32r + j] = p_InMatrixA_f32r[pivot_element_row * v_nDim_BSX_S32r + j];
        p_InMatrixA_f32r[pivot_element_row * v_nDim_BSX_S32r + j] = swap;
      }
      swap = p_InMatrixB_f32r[i];
      p_InMatrixB_f32r[i] = p_InMatrixB_f32r[pivot_element_row];
      p_InMatrixB_f32r[pivot_element_row] = swap;
    }
    if (pivot_element_value == 0.0) {
      return -1;
    }
    for (j = i + 1; j < v_nDim_BSX_S32r; j++) {
      BSX_F32 scale_factor = - p_InMatrixA_f32r[j * v_nDim_BSX_S32r + i] / pivot_element_value;
      for (k = i; k < v_nDim_BSX_S32r; k++) {
        p_InMatrixA_f32r[j * v_nDim_BSX_S32r + k] += scale_factor * p_InMatrixA_f32r[i * v_nDim_BSX_S32r + k];
      }
      p_InMatrixB_f32r[j] += scale_factor * p_InMatrixB_f32r[i];
    }
  }
  for (i = v_nDim_BSX_S32r - 1; i >= 0; i--) {
    p_OutMatrix_f32r[i] = p_InMatrixB_f32r[i];
    for (j = v_nDim_BSX_S32r - 1; j > i; j--) {
      p_OutMatrix_f32r[i] -= p_InMatrixA_f32r[i * v_nDim_BSX_S32r + j] * p_OutMatrix_f32r[j];
    }
    p_OutMatrix_f32r[i] /= p_InMatrixA_f32r[i * v_nDim_BSX_S32r + i];
  }
  return 0;
}

BSX_S8 MATRIX_Equal(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Row_u8r, BSX_U8 v_Column_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Row_u8r * (BSX_S32)v_Column_u8r;
  for (i = 0; i < size; i++) {
    BSX_F32 a = p_InMatrixA_f32r[i];
    BSX_F32 b = p_InMatrixB_f32r[i];
    if (a == b) {
      p_OutMatrix_f32r[i] = 1.0f;
    } else {
      p_OutMatrix_f32r[i] = 0.0f;
    }
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseSqrt(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = sqrtf(p_InMatrix_f32r[i]);
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseScalarSubtract(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r, BSX_U8 v_Type_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    if (v_Type_u8r == 2) {
      p_OutMatrix_f32r[i] = v_Value_f32r - p_InMatrix_f32r[i];
    } else {
      p_OutMatrix_f32r[i] = p_InMatrix_f32r[i] - v_Value_f32r;
    }
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseScalarPowerFixed(BSX_S16 *p_InMatrix_s16r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_PowerValue_u8r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i, j;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    BSX_F32 sum = 1.0f;
    BSX_F32 value = (BSX_F32)p_InMatrix_s16r[i];
    for (j = 0; j < (BSX_S32)v_PowerValue_u8r; j++) {
      sum *= value;
    }
    p_OutMatrix_f32r[i] = sum;
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseScalarPower(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_PowerValue_u8r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i, j;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    BSX_F32 sum = 1.0f;
    BSX_F32 value = p_InMatrix_f32r[i];
    for (j = 0; j < (BSX_S32)v_PowerValue_u8r; j++) {
      sum *= value;
    }
    p_OutMatrix_f32r[i] = sum;
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseMultiply(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrixA_f32r[i] * p_InMatrixB_f32r[i];
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseScalarDivision(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrix_f32r[i] / v_Value_f32r;
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseScalarAdd(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrix_f32r[i] + v_Value_f32r;
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseMultiplyFixed(BSX_S16 *p_InMatrixA_s16r, BSX_S16 *p_InMatrixB_s16r, BSX_S32 *p_OutMatrix_BSX_S32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_BSX_S32r[i] = (BSX_S32)p_InMatrixA_s16r[i] * (BSX_S32)p_InMatrixB_s16r[i];
  }
  return 0;
}

BSX_S8 MATRIX_ElementwiseScalarMultiply(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrix_f32r[i] * v_Value_f32r;
  }
  return 0;
}


BSX_S8 MATRIX_ElementwiseDivide(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrixA_f32r[i] / p_InMatrixB_f32r[i];
  }
  return 0;
}

BSX_F32 MATRIX_DiagonalSum(BSX_F32 *p_InMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_fromindex, BSX_U8 v_toindex) {
  BSX_S32 i;
  BSX_F32 sum = 0.0f;
  for (i = (BSX_S32)v_fromindex - 1; i < (BSX_S32)v_toindex; i++) {
    sum += p_InMatrix_f32r[i * ((BSX_S32)v_Rows_u8r + 1)];
  }
  return sum;
}

BSX_S8 MATRIX_Diagonal(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i, j;
  if (v_Rows_u8r == v_Columns_u8r) {
    for (i = 0; i < (BSX_S32)v_Rows_u8r; i++) {
      p_OutMatrix_f32r[i] = *p_InMatrix_f32r;
      //++p_OutMatrix_f32r;
      p_InMatrix_f32r += (BSX_S32)v_Rows_u8r + 1;
    }
    return 0;
  }
  if (v_Rows_u8r == 1 || v_Columns_u8r == 1) {
    if (v_Rows_u8r < v_Columns_u8r) {
      v_Rows_u8r = v_Columns_u8r;
    }
    for (i = 0; i < (BSX_S32)v_Rows_u8r; i++) {
      for (j = 0; j < (BSX_S32)v_Rows_u8r; j++) {
        BSX_F32 value = 0.0f;
        if (i == j) {
          value = *p_InMatrix_f32r;
        }
        p_OutMatrix_f32r[j] = value;
      }
      ++p_InMatrix_f32r;
      p_OutMatrix_f32r += v_Rows_u8r;
    }
  }
  return 0;
}

BSX_S8 MATRIX_CopyFromMatIndex(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_outStartR_u8r, BSX_U8 v_outStartC_u8r, BSX_U8 v_inRowSize_u8r, BSX_U8 v_inColSize_u8r, BSX_U8 v_outColSize_u8r) {
  BSX_S32 i, j;
  p_OutMatrix_f32r += (BSX_S32)v_outStartC_u8r + (BSX_S32)v_outStartR_u8r * (BSX_S32)v_outColSize_u8r;
  for (i = 0; i < (BSX_S32)v_inRowSize_u8r; i++) {
    for (j = 0; j < (BSX_S32)v_inColSize_u8r; j++) {
      p_OutMatrix_f32r[j] = p_InMatrix_f32r[j];
    }
    p_InMatrix_f32r += v_inColSize_u8r;
    p_OutMatrix_f32r += v_outColSize_u8r;
  }
  return 0;
}

BSX_S8 MATRIX_Copy(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_row_u8r * (BSX_S32)v_column_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrix_f32r[i];
  }
  return 0;
}

BSX_S8 MATRIX_AddFixed(BSX_S16 *p_InMatrixA_s16r, BSX_S16 *p_InMatrixB_s16r, BSX_S32 *p_OutMatrix_BSX_S32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_BSX_S32r[i] = (BSX_S32)p_InMatrixB_s16r[i] + (BSX_S32)p_InMatrixB_s16r[i];
  }
  return 0;
}

BSX_S8 MATRIX_Add(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    p_OutMatrix_f32r[i] = p_InMatrixA_f32r[i] + p_InMatrixB_f32r[i];
  }
  return 0;
}

BSX_S8 MATRIX_Absolute(BSX_S16 *p_InMatrix_s16r, BSX_S16 *p_OutMatrix_s16r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r) {
  BSX_S32 i;
  BSX_S32 size = (BSX_S32)v_Rows_u8r * (BSX_S32)v_Columns_u8r;
  for (i = 0; i < size; i++) {
    BSX_S16 value = p_InMatrix_s16r[i];
    if (value < 0) {
      value = -value;
    }
    p_OutMatrix_s16r[i] = value;
  }
  return 0;
}
