#include "InternalDataTypes.h"
ts_acc acc;
ts_gyro gyro;
ts_mag mag;
ts_tick s_tick;
ts_pedo pedo;

BSX_S8 bsx_get_version(ts_version *version) {
  version->major = 3;
  version->minor = 6;
  version->minorbugFix = 23;
  version->majorbugFix = 1;
  return 0;
}

BSX_S8 bsx_get_usecasecalltick(BSX_U8 *usecasetick) {
  *usecasetick = s_tick.usecase;
  return 0;
}

BSX_S8 bsx_get_stepeventdetectionopmode(BSX_U8 *eventstatus) {
  *eventstatus = pedo.eventdetection;
  return 0;
}
