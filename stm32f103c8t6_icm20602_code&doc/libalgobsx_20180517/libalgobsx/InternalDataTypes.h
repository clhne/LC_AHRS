#ifndef __BSXFUSIONLIBRARYINTERNALTYPES_H__
#define __BSXFUSIONLIBRARYINTERNALTYPES_H__
#include "GenericAccSpec.h"
#include "GenericGyroSpec.h"
#include "GenericMagSpec.h"

typedef struct {
  BSX_U8 general;
  BSX_U8 dataCorrection;
  BSX_U8 digitalFilter1;
  BSX_U8 digitalFilter2;
} ts_mode;

typedef struct {
  ts_mode mode;
  ts_axisconfig axisConfig;
  ts_dataxyz off;
  ts_sensmatrix sensM;
  BSX_F32 digFilt1ConstiirCoeff[10];
  BSX_F32 digFilt1Param;
  BSX_F32 digFilt2Param;
} ts_preProc;

typedef struct{
  ts_accsensorspec spec;
  /*ts_GenClockDomain clk;
  ts_GenericDataPreProc preProc;
  ts_AccelObserver s_accelObserver;
  BSX_U8 gravitySource;
  BSX_U8 linAccSource;
  BSX_U8 accDyn4DistDetMode;*/
} ts_acc;

typedef struct {
  ts_gyrosensorspec spec;
  /*ts_GenClockDomain clk;
  ts_GenericDataPreProc preProc;*/
} ts_gyro;

typedef struct {
  ts_magsensorspec spec;
  /*ts_GenClockDomain clk;
  ts_GenericDataPreProc preProc;
  BSX_U8 calibSource;*/
} ts_mag;

typedef struct {
  BSX_U8 opmode;
  BSX_U8 dataratetick;
  BSX_U8 eventdetection;
  BSX_U8 robuststatus;
  BSX_U8 V_pedoprocTick_U8R;
  BSX_U8 V_pedoDatarate_U8R;
  BSX_U32 V_trigPedoTimerVal_U32R;
} ts_pedo;


#endif