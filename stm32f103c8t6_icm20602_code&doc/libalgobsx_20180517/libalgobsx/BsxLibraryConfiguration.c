#include "BsxLibraryConfiguration.h"
BSX_U8 bsx_copyBytefromMemory(BSX_U8 *dst, BSX_U8 *src, BSX_S32 length) {
  int i;
  for (i = 0; i < length; i++ ) {
    *(dst + i) = *(src + i);
  }
  return 0;
}

BSX_U8 bsx_crc(BSX_U8* spec) {
  BSX_U16 length = *((BSX_U16 *)spec);
  BSX_S32 crc1_check = *(spec + length - 2);
  BSX_S32 crc2_check = *(spec + length - 1);
  BSX_S32 crc1 = 0;
  BSX_U32 crc2 = 0;
  int i = 0;
  BSX_S8 sum = 0;
  while (i < length - 2) {
    BSX_S8 val = *(spec + i++);
    sum += val;
    crc1 = (BSX_U8)(crc1 + sum);
    crc2 = (BSX_U8)((((BSX_U8)crc2 << 7) | (crc2 >> 1)) + val);
  } 
  if (crc1 == crc1_check && crc2 == crc2_check) {
    return 0;
  }
  return 1;
}