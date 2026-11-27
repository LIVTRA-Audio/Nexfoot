/* Modified for Nexfoot; upstream notices are retained. */
#ifndef MD400_PRESET_STORAGE_H
#define MD400_PRESET_STORAGE_H
#include "typedef.h"
int md400_preset_storage_init(void *scratch, u16 capacity);
int md400_preset_storage_read(u16 id, void *data, u16 len);
int md400_preset_storage_write(u16 id, void *data, u16 len);
#endif
