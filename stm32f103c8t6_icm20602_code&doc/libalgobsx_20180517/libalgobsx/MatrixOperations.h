#ifndef __MATRIXOPERATIONS_H__
#define __MATRIXOPERATIONS_H__
#include "BsxLibraryDataTypes.h"
BSX_S8 MATRIX_Transpose(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_Subtract(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_Multiply_s32(BSX_S32 *p_InMatrixA_s32r, BSX_S32 *p_InMatrixB_s32r, BSX_S32 *p_OutMatrix_s32r, BSX_U8 v_RowsA_u8r, BSX_U8 v_RowsB_u8r, BSX_U8 v_ColumnsB_u8r);
BSX_S8 MATRIX_Multiply(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_RowsA_u8r, BSX_U8 v_RowsB_u8r, BSX_U8 v_ColumnsB_u8r);
BSX_S8 MATRIX_MeanFixed(BSX_S16 *p_InMatrix_s16r, BSX_S16 *p_OutMatrix_s16r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r);
BSX_S8 MATRIX_Mean(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r);
BSX_S8 MATRIX_Inverse(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_dim_u8r, BSX_U8 v_LSEMethod_u8r);
BSX_S8 MATRIX_Initialize_s32(BSX_S32 *p_InMatrix_s32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r, BSX_S32 v_initValue_s16r);
BSX_S8 MATRIX_InitializeFixed(BSX_S16 *p_InMatrix_s16r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r, BSX_S16 v_initValue_s16r);
BSX_S8 MATRIX_Initialize(BSX_F32 *p_InMatrix_f32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r, BSX_F32 v_initValue_f32r);
BSX_S8 MATRIX_Identity(BSX_F32 *p_InMatrix_f32r, BSX_U8 v_dim_u8r);
BSX_S8 MATRIX_GaussJordanElimination(BSX_U8 v_rows, BSX_U8 v_colA, BSX_U8 v_colB, BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r);
BSX_S8 MATRIX_GaussElimination(BSX_S32 v_nDim_BSX_S32r, BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r);
BSX_S8 MATRIX_Equal(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Row_u8r, BSX_U8 v_Column_u8r);
BSX_S8 MATRIX_ElementwiseSqrt(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseScalarSubtract(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r, BSX_U8 v_Type_u8r);
BSX_S8 MATRIX_ElementwiseScalarPowerFixed(BSX_S16 *p_InMatrix_s16r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_PowerValue_u8r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseScalarPower(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_PowerValue_u8r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseMultiply(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseScalarDivision(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseScalarAdd(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseMultiplyFixed(BSX_S16 *p_InMatrixA_s16r, BSX_S16 *p_InMatrixB_s16r, BSX_S32 *p_OutMatrix_BSX_S32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseScalarMultiply(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_F32 v_Value_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_ElementwiseDivide(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_F32 MATRIX_DiagonalSum(BSX_F32 *p_InMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_fromindex, BSX_U8 v_toindex);
BSX_S8 MATRIX_Diagonal(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_CopyFromMatIndex(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_outStartR_u8r, BSX_U8 v_outStartC_u8r, BSX_U8 v_inRowSize_u8r, BSX_U8 v_inColSize_u8r, BSX_U8 v_outColSize_u8r);
BSX_S8 MATRIX_Copy(BSX_F32 *p_InMatrix_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_row_u8r, BSX_U8 v_column_u8r);
BSX_S8 MATRIX_AddFixed(BSX_S16 *p_InMatrixA_s16r, BSX_S16 *p_InMatrixB_s16r, BSX_S32 *p_OutMatrix_BSX_S32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_Add(BSX_F32 *p_InMatrixA_f32r, BSX_F32 *p_InMatrixB_f32r, BSX_F32 *p_OutMatrix_f32r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
BSX_S8 MATRIX_Absolute(BSX_S16 *p_InMatrix_s16r, BSX_S16 *p_OutMatrix_s16r, BSX_U8 v_Rows_u8r, BSX_U8 v_Columns_u8r);
#endif
