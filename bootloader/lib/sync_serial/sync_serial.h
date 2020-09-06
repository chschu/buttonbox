#ifndef _SYNC_SERIAL_H
#define _SYNC_SERIAL_H

#include <inttypes.h>

// initialize synchronous serial communication
void sync_serial_init();

// reset modified registers
void sync_serial_reset();

// send a byte
void sync_serial_putc(uint8_t data);

// receive a byte
uint8_t sync_serial_getc();

#endif
