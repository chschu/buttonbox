#include <util/delay.h>
#include <i2cmaster.h>

#include "PCA9685.h"

PCA9685::PCA9685(uint8_t addr) {
    _addr = (addr & 0x3f) | 0x40;
}

void PCA9685::begin() {
    uint8_t sla_w = (_addr << 1) | I2C_WRITE;

    // put to sleep to configure prescaler
    i2c_start_wait(sla_w);
    i2c_write(REG_MODE1);
    i2c_write(MODE1_SLEEP);

    // configure prescaler
    i2c_rep_start(sla_w);
    i2c_write(REG_PRE_SCALE); // PRESCALE
    i2c_write(6); // 25000000 Hz / (4096 * x + 1) --> ~ 1017 Hz

    // output configuration (non-inverting, totem-pole, output low when /OE = 1)
    // this is the same as the default configuration
    i2c_rep_start(sla_w);
    i2c_write(REG_MODE2);
    i2c_write(MODE2_OUTDRV);

    // wake up (set restart bit and clear sleep bit) and enable auto-increment
    i2c_rep_start(sla_w);
    i2c_write(REG_MODE1);
    i2c_write(MODE1_RESTART | MODE1_AI);

    i2c_stop();

    // wait 500 microseconds for the oscillator to stabilize 
    _delay_us(500);
}

void PCA9685::set(uint8_t led, uint16_t value) {
    uint16_t on, off;

    if (value >= 4095) {
        // fully on
        on = 4096;
        off = 0;
    } else if (value == 0) {
        // fully off
        on = 0;
        off = 4096;
    } else {
        on = 0;
        off = value;
    }

    uint8_t sla_w = (_addr << 1) | I2C_WRITE;

    i2c_start_wait(sla_w);
    i2c_write(REG_LED0_ON_L + 4 * led);
    i2c_write(on);
    i2c_write(on >> 8);
    i2c_write(off);
    i2c_write(off >> 8);
    i2c_stop();
}
