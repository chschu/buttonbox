#ifndef _PCA9658_H
#define _PCA9658_H

#include <inttypes.h>

class PCA9685 {
public:
    // only lower 6 bits of addr are used
    PCA9685(uint8_t addr = 0);

    void begin();
    void set(uint8_t led, uint16_t val);

private:
    uint8_t _addr;

    static const uint8_t REG_MODE1     = 0x00;
    static const uint8_t REG_MODE2     = 0x01;
    static const uint8_t REG_LED0_ON_L = 0x06; // followed by LED0_ON_H, LED0_OFF_L, LED0_OFF_H, LED1_ON_H, ...
    static const uint8_t REG_PRE_SCALE = 0xfe;

    static const uint8_t MODE1_RESTART = 0x80;
    static const uint8_t MODE1_EXTCLK  = 0x40;
    static const uint8_t MODE1_AI      = 0x20; // auto-increment register addresses
    static const uint8_t MODE1_SLEEP   = 0x10;
    static const uint8_t MODE1_SUB1    = 0x08;
    static const uint8_t MODE1_SUB2    = 0x04;
    static const uint8_t MODE1_SUB3    = 0x02;
    static const uint8_t MODE1_ALLCALL = 0x01;

    static const uint8_t MODE2_INVRT   = 0x10;
    static const uint8_t MODE2_OCH     = 0x08; // outputs change on last ACK (1) or on STOP (0)
    static const uint8_t MODE2_OUTDRV  = 0x04;
    static const uint8_t MODE2_OUTNE1  = 0x02; // output high impedance if ~OE = 1
    static const uint8_t MODE1_OUTNE0  = 0x01; // output if ~OE = 1 (ignored if OUTNE1 = 1)
};

#endif
