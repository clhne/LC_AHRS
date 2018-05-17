#ifndef __GENERICCLOCKDOMAIN_H__
#define __GENERICCLOCKDOMAIN_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_U8 V_sysProcTick_U8R;
  BSX_U8 V_calProcTick_U8R;
  BSX_U8 V_orientProcTick_U8R;
  BSX_U8 V_observerProcTick_u8r;
  BSX_U32 V_dT_U32R;
  BSX_U8 V_opmode_U8R;
  BSX_U8 V_systemDatarate_U8R;
  BSX_U8 V_calibDatarate_U8R;
  BSX_U8 V_orientDatarate_U8R;
  BSX_U8 V_observerDatarate_U8R;
  BSX_U32 V_currentTimerVal_U32R;
  BSX_U32 V_trigSysTimerVal_U32R;
  BSX_U32 V_trigCalTimerVal_U32R;
  BSX_U32 V_triOrientTimerVal_U32R;
  BSX_U32 V_triObserverTimerVal_U32R;
} ts_GenClockDomain;
BSX_S8 GENERICCLOCKDOMAIN_SetSystemDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r);
BSX_S8 GENERICCLOCKDOMAIN_SetOrientDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r);
BSX_S8 GENERICCLOCKDOMAIN_SetOperationMode(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_mode_u8r);
BSX_S8 GENERICCLOCKDOMAIN_SetObserverDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r);
BSX_S8 GENERICCLOCKDOMAIN_SetCalibDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r);
BSX_S8 GENERICCLOCKDOMAIN_Reset(ts_GenClockDomain *p_genClockDomain);
BSX_S8 GENERICCLOCKDOMAIN_Init(ts_GenClockDomain *p_genClockDomain);
BSX_S8 GENERICCLOCKDOMAIN_GetVersion(ts_version *p_version);
BSX_S8 GENERICCLOCKDOMAIN_GetSystemProcessingTimerTick(ts_GenClockDomain *p_genClockDomain, BSX_U8 *p_tickValue_u8r);
BSX_S8 GENERICCLOCKDOMAIN_GetOrientProcessingTimerTick(ts_GenClockDomain *p_genClockDomain, BSX_U8 *p_tickValue_u8r);
BSX_S8 GENERICCLOCKDOMAIN_GetCalibProcessingTimerTick(ts_GenClockDomain *p_genClockDomain, BSX_U8 *p_tickValue_u8r);
BSX_S8 GENERICCLOCKDOMAIN_DoStep(ts_GenClockDomain *p_genClockDomain, BSX_U32 v_timerValue_U32r, BSX_U8 *p_sysProcTick_u8r, BSX_U8 *p_calProc_u8r, BSX_U8 *p_orientProcTick_u8r, BSX_U8 *p_observerProcTick_u8r, BSX_U32 *p_dT_U32r);
#endif