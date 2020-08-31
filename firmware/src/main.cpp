#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/cpufunc.h>

#include <i2cmaster.h>
#include <uart.h>

#include <Cycles.h>
#include <Debouncer.h>
#include <Blinker.h>
#include <PCA9685.h>
#include <MCP23017.h>

#include "transform.h"

const uint8_t CONNECTOR_COUNT = 16;
const uint32_t BOUNCE_CYCLES = MICROSECONDS_TO_CYCLES(10000);

// splitting macros for first byte (command + led) of the serial protocol
#define CMD(b) ((b) & 0xF0)
#define LED(b) ((b) & 0x0F)

// 0x0N: Button N activated (on release)
const uint8_t CMD_BUTTON = 0x00;

// 0x0N: LED N off
const uint8_t CMD_OFF = 0x00;

// 0x1N: LED N on
const uint8_t CMD_ON = 0x10;

// configuration for initialization animation
const uint32_t INIT_BLINK_OFFSET_CYCLES = MICROSECONDS_TO_CYCLES(50000);
const uint32_t INIT_BLINK_PERIOD_CYCLES = MICROSECONDS_TO_CYCLES(500000);

// configure blinking
const float BRIGHT_PHASE = 0.5f;
const float DARK_PHASE = 0.9f;
const uint32_t BLINK_PERIOD_CYCLES = MICROSECONDS_TO_CYCLES(500000);

// special blink period to indicate config mode
const uint32_t CONFIG_MODE_BLINK_PERIOD_CYCLES = MICROSECONDS_TO_CYCLES(200000);

class PCA9685Blinker : public Blinker {
public:
    void begin(PCA9685 *pca9685, uint8_t pin) {
        _pca9685 = pca9685;
        _pin = pin;
        Blinker::begin(cycles_get());
    }

    void update() {
        Blinker::update(cycles_get());
    }

protected:
    void render(float value) override {
        uint16_t cur = 4095 * Transform::gamma<11, 4>(value);
        _pca9685->set(_pin, cur);
    }

private:
    PCA9685 *_pca9685;
    uint8_t _pin;
};

PCA9685 pca9685;
MCP23017 mcp23017;

// indexes are physical connector numbers
PCA9685Blinker blinkers[CONNECTOR_COUNT];
Debouncer debouncers[CONNECTOR_COUNT];

// bijective mapping from logical value (0-15) to physical connector number (0-15)
// stored in EEPROM
// logical values are used for in serial communication and for the initialization animation
// unused logical values are represented by 255
// after the first 255 in this array, there may not be any other value
// all values other than 255 must be unique
uint8_t eepromConnectorForLogicalValue[CONNECTOR_COUNT] EEMEM = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

// copy of eepromConnectorForLogicalValue in RAM
// initialized from EEPROM during setup
uint8_t connectorForLogicalValue[CONNECTOR_COUNT] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

// number of used connectors
// this is the index of first 255 in connectorForLogicalValue
// initialized from EEPROM during setup
uint8_t usedConnectors = 0;

// inverse of connectorForLogicalValue
// unused connectors are represented by 255
// initialized from EEPROM during setup
uint8_t logicalValueForConnector[CONNECTOR_COUNT] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

bool configMode;
uint8_t nextLogicalValue;

// config mode is enabled by bridging the two center pins of the ISP connector (MOSI and SCK) at startup
bool checkConfigModePinsBridged() {
    // configure PB3 (MOSI) as input, and enable internal pull-up
    DDRB &= ~(1 << DDB3);
    PORTB |= (1 << PORTB3);

    // configure PB5 (SCK) as output
    DDRB |= (1 << DDB5);

    bool result = true;

    uint8_t testMask = 0b01011010;
    for (int i = 0; i < 8; i++) {
        uint8_t bit = (testMask >> i) & 1;

        // output bit on PB5
        if (bit) {
            PORTB = PORTB | (1 << PORTB5);
        } else {
            PORTB = PORTB & ~(1 << PORTB5);
        }

        // wait for one cycle (~67.8ns at 14.7456 MHz)
        _NOP();

        // check if bit is available on PB3
        if (((PINB >> PINB3) & 1) != bit) {
            // no, it's not - we're not in config mode
            result = false;
            break;
        }
    }

    return result;
}

int main() {
    cycles_init();

    // enter config mode if no connectors have been configured yet or the center ISP pins are bridged
    configMode = eeprom_read_byte(eepromConnectorForLogicalValue) >= CONNECTOR_COUNT || checkConfigModePinsBridged();

    if (configMode) {
        // clear (i.e. set to 255) all logical value mappings in EEPROM
        eeprom_update_block(connectorForLogicalValue, eepromConnectorForLogicalValue, sizeof(connectorForLogicalValue));
    } else {
        // read logical value mapping from EEPROM
        eeprom_read_block(connectorForLogicalValue, eepromConnectorForLogicalValue, sizeof(connectorForLogicalValue));

        // determine logical values for connectors
        for (int lv = 0; lv < CONNECTOR_COUNT; lv++) {
            uint8_t cn = connectorForLogicalValue[lv];
            if (cn < CONNECTOR_COUNT) {
                logicalValueForConnector[cn] = lv;
                usedConnectors++;
            }
        }
    }

    i2c_init();

    // perform software reset of I2C devices in case something is stuck
    i2c_start_wait((0x00 << 1) | I2C_WRITE);
    i2c_write(0x06);
    i2c_stop();

    // initialize MCP23017 and PCA9685
    mcp23017.begin();
    pca9685.begin();

    // initialize output blinkers
    for (int cn = 0; cn < CONNECTOR_COUNT; cn++) {
        blinkers[cn].begin(&pca9685, cn);
    }

    // initialize input debouncers
    uint16_t inputs = mcp23017.read();
    for (int cn = 0; cn < CONNECTOR_COUNT; cn++) {
        debouncers[cn].begin((inputs >> cn) & 1, BOUNCE_CYCLES);
    }

    // pull ~OE low to enable LEDs
    DDRC |= (1 << DDC2);
    PORTC &= ~(1 << PORTC2);

    if (configMode) {
        // indicate config mode
        for (int cn = 0; cn < CONNECTOR_COUNT; cn++) {
            blinkers[cn].blink(CONFIG_MODE_BLINK_PERIOD_CYCLES);
        }
    } else {
        // perform initialization animation
        uint32_t cycles0 = cycles_get();
        uint32_t duration;
        do {
            duration = cycles_get() - cycles0;
            // start blinking according to the logical values
            uint32_t lv = duration / INIT_BLINK_OFFSET_CYCLES;
            if (lv < usedConnectors) {
                uint8_t cn = connectorForLogicalValue[lv];
                blinkers[cn].blink(INIT_BLINK_PERIOD_CYCLES);
            }

            for (uint8_t lv = 0; lv < usedConnectors; lv++) {
                uint8_t cn = connectorForLogicalValue[lv];
                blinkers[cn].update();
                if (!blinkers[cn].isOff()) {
                    blinkers[cn].stopAtPhase(DARK_PHASE);
                }
            }
        } while (duration < usedConnectors * INIT_BLINK_OFFSET_CYCLES + INIT_BLINK_PERIOD_CYCLES);

        uart_init(UART_BAUD_SELECT(115200, F_CPU));
    }

    if (configMode) {
        for (;;) {
            uint16_t inputs = mcp23017.read();
            for (uint8_t cn = 0; cn < CONNECTOR_COUNT; cn++) {
                blinkers[cn].update();
                if (debouncers[cn].update((inputs >> cn) & 1, cycles_get()) && debouncers[cn].get() && logicalValueForConnector[cn] >= CONNECTOR_COUNT) {
                    logicalValueForConnector[cn] = usedConnectors;
                    connectorForLogicalValue[usedConnectors] = cn;

                    // store new logical value mapping in EEPROM
                    eeprom_update_byte(&eepromConnectorForLogicalValue[usedConnectors], cn);

                    usedConnectors++;

                    blinkers[cn].stopAtPhase(BRIGHT_PHASE);
                }
            }
        }
    } else {
        for (;;) {
            unsigned int c;
            while ((c = uart_getc()) != UART_NO_DATA) {
                uint8_t lv = LED(c);
                uint8_t cn = connectorForLogicalValue[lv];
                if (cn < CONNECTOR_COUNT) {
                    switch (CMD(c)) {
                    case CMD_OFF:
                        blinkers[cn].stopAtPhase(DARK_PHASE);
                        break;
                    case CMD_ON:
                        blinkers[cn].stopAtPhase(BRIGHT_PHASE);
                        break;
                    }
                }
            }

            uint16_t inputs = mcp23017.read();
            for (uint8_t lv = 0; lv < usedConnectors; lv++) {
                uint8_t cn = connectorForLogicalValue[lv];
                blinkers[cn].update();
                if (debouncers[cn].update((inputs >> cn) & 1, cycles_get()) && !blinkers[cn].isBlinking() && debouncers[cn].get()) {
                    blinkers[cn].blink(BLINK_PERIOD_CYCLES);
                    uart_putc(CMD_BUTTON | lv);
                }
            }
        }
    }
}
