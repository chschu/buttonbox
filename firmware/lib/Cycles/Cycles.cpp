#include <avr/interrupt.h>
#include <avr/io.h>

#include "Cycles.h"

volatile uint16_t _cycles_high = 0;

ISR (TIMER1_OVF_vect) {
    _cycles_high++;
}

void cycles_init() {
    // make Timer1 run at full CPU frequency
    TCCR1B = _BV(CS10);

    // enable overflow interrupt
    TIMSK1 |= _BV(TOIE1);

    // enable global interrupts
    sei();
}

uint32_t cycles_get() {
    uint8_t prevSREG = SREG;

    // disable interrupts for a consistent read of _cycles_high and TCNT1
    cli();

    // determine low and high word of cycle count, as well as timer interrupt flags
    uint16_t cycles_low = TCNT1;
    uint16_t cycles_high = _cycles_high;
    uint16_t flags = TIFR1;

    // restore interrupt flag
    SREG = prevSREG;

    // check if there is a pending timer overflow and TCNT1 had already wrapped around before it was read
    if ((flags & _BV(TOV1)) && cycles_low < 0x8000) {
        cycles_high++;
    }

    return ((uint32_t) cycles_high << 16) | cycles_low;
}
