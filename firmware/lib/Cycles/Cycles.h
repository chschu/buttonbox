#ifndef _CYCLES_H
#define _CYCLES_H

#include <inttypes.h>

#define MICROSECONDS_TO_CYCLES(m) ((uint32_t) ((uint64_t) (m) * F_CPU / 1000000))

void cycles_init();
uint32_t cycles_get();

#endif
