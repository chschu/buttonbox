#include <i2cmaster.h>

#include "MCP23017.h"

MCP23017::MCP23017(uint8_t addr) {
    _addr = (addr & 0x07) | 0x20;
}

void MCP23017::begin() {
    uint8_t sla_w = (_addr << 1) | I2C_WRITE;

    // enable byte mode (toggles between A/B registers if IOCON.BANK = 0)
    i2c_start_wait(sla_w);
    i2c_write(REG_IOCONA);
    i2c_write(IOCON_SEQOP);
    i2c_stop(); // MCP23017 does not support restart condition for writing!

    // configure all pins as inputs
    i2c_start_wait(sla_w);
    i2c_write(REG_IODIRA);
    i2c_write(0xff);
    i2c_write(0xff);
    i2c_stop();

    // configure pull-up resistors for all pins
    i2c_start_wait(sla_w);
    i2c_write(REG_GPPUA);
    i2c_write(0xff);
    i2c_write(0xff);
    i2c_stop();
}

uint16_t MCP23017::read() {
    uint8_t sla_w = (_addr << 1) | I2C_WRITE;
    uint8_t sla_r = (_addr << 1) | I2C_READ;

    i2c_start_wait(sla_w);
    i2c_write(REG_GPIOA);

    i2c_rep_start(sla_r);
    uint8_t a = i2c_readAck();
    uint16_t b = i2c_readNak();

    i2c_stop();

    return (b << 8) | a;
}
