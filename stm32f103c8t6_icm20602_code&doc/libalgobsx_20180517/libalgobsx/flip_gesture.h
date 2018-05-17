#ifndef __FLIP_GESTURE_H__
#define __FLIP_GESTURE_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_U16 faceup;
  BSX_U16 facedown;
  BSX_U16 neutral;
} flip_count_t;
typedef struct {
  flip_t flip_thres;
  flip_count_t flipcount;
  BSX_F32 hyst_thres;
  BSX_U8 flip_status;
  BSX_U16 flipcount_thres;
  BSX_U16 ref_time;
  BSX_U16 detect_time;
} flipgesture_t;
BSX_S8 flipgesture_set_flipstate_detecttime(BSX_U16 det_time, BSX_U16 ref_time);
BSX_S8 flipgesture_set_flipangle(flip_t flipangle);
BSX_S8 flipgesture_run(ts_dataxyzf32 accdata, BSX_U8 onegflag);
BSX_S8 flipgesture_reset();
BSX_S8 flipgesture_init();
BSX_S8 flipgesture_get_flipstatus(BSX_U8 *flip_status);
BSX_S8 flipgesture_get_flipstate_detecttime(BSX_U16 *time);
BSX_S8 flipgesture_get_flipangle(flip_t *flipangle);
#endif