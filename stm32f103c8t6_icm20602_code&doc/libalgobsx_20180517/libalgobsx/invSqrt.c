#include <math.h>
#include "invSqrt.h"
BSX_F32 invSqrtF(BSX_F32 x) {
  BSX_F32 y;
  if (x >= 1.18e-38) {
    BSX_F32 halfx = 0.5f * x;
    BSX_U32 i = *(BSX_U32 *)&x;
    i = (BSX_U32)0x5F375A86 - (i >> 1);
    y = *(BSX_F32 *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
  } else if (x <= 0.0) {
    BSX_U32 i = 0xFFFFFFFF;
    y = *(BSX_F32 *)&i;
  } else {
    y = 1.0f / sqrtf(x);
  }
  return y;
}

BSX_F64 invSqrt(BSX_F64 x) {
  BSX_F64 y;
  if (x >= 2.23e-308) {
    BSX_F64 halfx = 0.5 * x;
    BSX_U64 i = *(BSX_U64 *)&x;
    i = (BSX_U64)0x5FE6EC85E7DE30DA - (i >> 1);
    y = *(BSX_F64 *)&i;
    y = y * (1.5 - (halfx * y * y));
    y = y * (1.5 - (halfx * y * y));
    y = y * (1.5 - (halfx * y * y));
    y = y * (1.5 - (halfx * y * y));
    y = y * (1.5 - (halfx * y * y));
  } else if (x <= 0.0) {
    BSX_U64 i = 0xFFFFFFFFFFFFFFFF;
    y = *(BSX_F64 *)&i;
  } else {
    y = 1.0 / sqrt(x);
  }
  return y;
}
