#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MCP23017.h>
#include <Debouncer.h>

#include <Blinker.h>

#include <avr/eeprom.h>

#include "transform.h"

#define CONNECTOR_COUNT 16
#define BOUNCE_MILLIS 10

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
const unsigned long long INIT_BLINK_OFFSET_MILLIS = 50;
const uint16_t INIT_BLINK_PERIOD_MILLIS = 500;

// configure blinking
const float BRIGHT_PHASE = 0.5f;
const float DARK_PHASE = 0.9f;
const uint16_t BLINK_PERIOD_MILLIS = 500;

// special blink period to indicate config mode
const uint16_t CONFIG_MODE_BLINK_PERIOD_MILLIS = 200;

class PCA9685Blinker : public Blinker {
public:
    void begin(Adafruit_PWMServoDriver *pca9685, uint8_t pin) {
        _pca9685 = pca9685;
        _pin = pin;
        Blinker::begin(micros());
    }

    void update() {
        Blinker::update(micros());
    }

protected:
    void render(float value) override {
        uint16_t cur = 4095 * Transform::gamma<11, 4>(value);
        _pca9685->setPin(_pin, cur);
    }

private:
    Adafruit_PWMServoDriver *_pca9685;
    uint8_t _pin;
};

Adafruit_PWMServoDriver pca9685;
Adafruit_MCP23017 mcp23017;

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

void setup() {
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

    // initialize MCP23017 and PCA9685
    mcp23017.begin();
    pca9685.begin();

    // clock must be set after the above "begin" calls, because they set it to 100 kHz
    Wire.setClock(800000);

    // initialize output blinkers
    for (int cn = 0; cn < CONNECTOR_COUNT; cn++) {
        blinkers[cn].begin(&pca9685, cn);

        mcp23017.pinMode(cn, INPUT);
        mcp23017.pullUp(cn, HIGH);
    }

    // initialize input debouncers
    uint16_t inputs = mcp23017.readGPIOAB();
    for (int cn = 0; cn < CONNECTOR_COUNT; cn++) {
        debouncers[cn].begin((inputs >> cn) & 1, BOUNCE_MILLIS);
    }

    // pull ~OE low to enable LEDs
    DDRC |= (1 << DDC2);
    PORTC &= ~(1 << PORTC2);

    if (configMode) {
        // indicate config mode
        for (int cn = 0; cn < CONNECTOR_COUNT; cn++) {
            blinkers[cn].blink(CONFIG_MODE_BLINK_PERIOD_MILLIS);
        }
    } else {
        // perform initialization animation
        unsigned long millis0 = millis();
        unsigned long duration;
        do {
            duration = millis() - millis0;
            // start blinking according to the logical values
            unsigned long lv = duration / INIT_BLINK_OFFSET_MILLIS;
            if (lv < usedConnectors) {
                uint8_t cn = connectorForLogicalValue[lv];
                blinkers[cn].blink(INIT_BLINK_PERIOD_MILLIS);
            }

            for (uint8_t lv = 0; lv < usedConnectors; lv++) {
                uint8_t cn = connectorForLogicalValue[lv];
                blinkers[cn].update();
                if (!blinkers[cn].isOff()) {
                    blinkers[cn].stopAtPhase(DARK_PHASE);
                }
            }
        } while (duration < usedConnectors * INIT_BLINK_OFFSET_MILLIS + INIT_BLINK_PERIOD_MILLIS);
    }

    Serial.begin(115200);
}

void loop() {
    if (configMode) {
        uint16_t inputs = mcp23017.readGPIOAB();
        for (uint8_t cn = 0; cn < CONNECTOR_COUNT; cn++) {
            blinkers[cn].update();
            if (debouncers[cn].update((inputs >> cn) & 1, micros()) && debouncers[cn].get() && logicalValueForConnector[cn] >= CONNECTOR_COUNT) {
                logicalValueForConnector[cn] = usedConnectors;
                connectorForLogicalValue[usedConnectors] = cn;

                // store new logical value mapping in EEPROM
                eeprom_update_byte(&eepromConnectorForLogicalValue[usedConnectors], cn);

                usedConnectors++;

                blinkers[cn].stopAtPhase(BRIGHT_PHASE);
            }
        }
    } else {
        int c;
        while ((c = Serial.read()) >= 0) {
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

        uint16_t inputs = mcp23017.readGPIOAB();
        for (uint8_t lv = 0; lv < usedConnectors; lv++) {
            uint8_t cn = connectorForLogicalValue[lv];
            blinkers[cn].update();
            if (debouncers[cn].update((inputs >> cn) & 1, micros()) && !blinkers[cn].isBlinking() && debouncers[cn].get()) {
                blinkers[cn].blink(BLINK_PERIOD_MILLIS);
                Serial.write(CMD_BUTTON | lv);
            }
        }
    }
}
