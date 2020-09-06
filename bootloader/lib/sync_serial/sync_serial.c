#include <inttypes.h>

#include <avr/io.h>

#include "sync_serial.h"

void sync_serial_init() {
    // set baud rate
    UBRR0L = (((F_CPU) + 8UL * (BAUD_RATE)) / (16UL * (BAUD_RATE)) - 1UL);

    // enable both directions
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
}

void sync_serial_reset() {
    // reset modified registers (all writable bits have a default of 0)
    UCSR0A = 0;
    UCSR0B = 0;
    UBRR0L = 0;
    UDR0 = 0;
}

void sync_serial_putc(uint8_t c) {
    // write data to be send
    UDR0 = c;

    // wait until ready
    while (!(UCSR0A & _BV(TXC0)));

    // clear transmit  flag by writing 1 to it
    UCSR0A |= _BV(TXC0);
}

uint8_t sync_serial_getc() {
    // wait for data
    while (!(UCSR0A & _BV(RXC0)));

    // return received data
    return UDR0;
}
