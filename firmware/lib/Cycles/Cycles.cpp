#include <avr/interrupt.h>
#include <avr/io.h>

#include "Cycles.h"

volatile uint32_t _cycles = 0;

ISR (TIMER0_OVF_vect) {
    _cycles += 256;
}

void cycles_init() {
    // make Timer0 run at full CPU frequency
    TCCR0B = (1<<CS00);

    // enable overflow interrupt
    TIMSK0 |= (1<<TOIE0);

    // enable global interrupts
    sei();
}

uint32_t cycles_get() {
    uint8_t prevSREG = SREG;

    // disable interrupts for a consistent read of _cycles
    cli();
    uint32_t result = _cycles + TCNT0;

    // restore interrupt flag
    SREG = prevSREG;

    return result;
}
