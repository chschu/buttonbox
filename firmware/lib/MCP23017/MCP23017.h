#ifndef _MCP23017_H
#define _MCP23017_H

class MCP23017 {
public:
    // only lower 3 bits of addr are used
    MCP23017(uint8_t addr = 0);

    void begin();
    uint16_t read();

private:
    uint8_t _addr;

    // Registers when IOCON.BANK = 0 (default)
    const uint8_t REG_IODIRA = 0x00;
    const uint8_t REG_IODIRB = 0x01;
    const uint8_t REG_IPOLA = 0x02;
    const uint8_t REG_IPOLB = 0x03;
    const uint8_t REG_GPINTENA = 0x04;
    const uint8_t REG_GPINTENB = 0x05;
    const uint8_t REG_DEFVALA = 0x06;
    const uint8_t REG_DEFVALB = 0x07;
    const uint8_t REG_INTCONA = 0x08;
    const uint8_t REG_INTCONB = 0x09;
    const uint8_t REG_IOCONA = 0x0a;
    const uint8_t REG_IOCONB = 0x0b;
    const uint8_t REG_GPPUA = 0x0c;
    const uint8_t REG_GPPUB = 0x0d;
    const uint8_t REG_INTFA = 0x0e;
    const uint8_t REG_INTFB = 0x0f;
    const uint8_t REG_INTCAPA = 0x10;
    const uint8_t REG_INTCAPB = 0x11;
    const uint8_t REG_GPIOA = 0x12;
    const uint8_t REG_GPIOB = 0x13;
    const uint8_t REG_OLATA = 0x14;
    const uint8_t REG_OLATB = 0x14;

    const uint8_t IOCON_BANK   = 0x80; // alternating (0, default, ABAB...) or grouped (1, AA...BB...) registers
    const uint8_t IOCON_MIRROR = 0x40; // separate (0, default) or mirrored (1) INTA and INTB pins
    const uint8_t IOCON_SEQOP  = 0x20; // sequential mode (0, default) or byte mode (1)
    const uint8_t IOCON_DISSLW = 0x10; // slew rate control of SDA pin enabled (0, default) or disabled (1)  
    // IOCON.HAEN is only usable for MCP23S17
    const uint8_t IOCON_ODR    = 0x04; // totem-pole (0, default) or open-drain (1) INTA and INTB pins
    const uint8_t IOCON_INTPOL = 0x02; // active-low (0, default) or active-high (1) INTA and INTB pins (ignored if IOCON.ODR = 1)
};

#endif
