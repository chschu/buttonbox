#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MCP23017.h>
#include <Debouncer.h>

#include <Blinker.h>

#include "transform.h"

#define LED_COUNT 16
#define BOUNCE_MILLIS 10

// splitting macros for first byte (command + led) of the serial protocol
#define CMD(b) ((b) & 0xF0)
#define LED(b) ((b) & 0x0F)

// 0x4N: LED N off / button N up
const uint8_t CMD_OFF = 0x40;

// 0x5N: LED N on / BUTTON N down
const uint8_t CMD_ON = 0x50;

// 0x6N 0xHH 0xLL: LED N blink a period of 0xHHLL milliseconds
const uint8_t CMD_BLINK = 0x60;

#if LED_COUNT >= 1 && LED_COUNT <= 16
const uint16_t ALL_LED_BITS = (1L << LED_COUNT) - 1;
#else
#error LED_COUNT must be a value from 1 to 16.
#endif

// configuration for initialization animation
const unsigned long long INIT_BLINK_OFFSET_MILLIS = 50;
const uint16_t INIT_BLINK_PERIOD_MILLIS = 500;

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

class MCP23017Debouncer : public Debouncer {
public:
    void begin(Adafruit_MCP23017 *mcp23017, uint8_t pin, uint16_t bounceMillis) {
        _mcp23017 = mcp23017;
        _pin = pin;
        Debouncer::begin(_mcp23017->digitalRead(_pin), bounceMillis);
    }

    bool update() {
        return Debouncer::update(_mcp23017->digitalRead(_pin), micros());
    }

private:
    Adafruit_MCP23017 *_mcp23017;
    uint8_t _pin;
};

Adafruit_PWMServoDriver pca9685;
Adafruit_MCP23017 mcp23017;

PCA9685Blinker blinkers[LED_COUNT];
MCP23017Debouncer debouncers[LED_COUNT];

void setup() {
    // initialize MCP23017 and PCA9685
    mcp23017.begin();
    pca9685.begin();

    // clock must be set after the above "begin" calls, because they set it to 100 kHz
    Wire.setClock(400000);

    // initialize output blinkers and input debouncers
    for (int i = 0; i < LED_COUNT; i++) {
        blinkers[i].begin(&pca9685, i);

        mcp23017.pinMode(i, INPUT);
        mcp23017.pullUp(i, HIGH);
        debouncers[i].begin(&mcp23017, i, BOUNCE_MILLIS);
    }

    // pull ~OE low to enable LEDs
    DDRC |= (1 << PC2);
    PORTC &= ~(1 << PC2);

    // perform initialization animation
    unsigned long millis0 = millis();
    unsigned long duration;
    do {
        duration = millis() - millis0;
        int p = duration / INIT_BLINK_OFFSET_MILLIS;
        if (p < LED_COUNT) {
            blinkers[p].blink(INIT_BLINK_PERIOD_MILLIS);
        }

        for (uint8_t i = 0; i < LED_COUNT; i++) {
            blinkers[i].update();
            if (!blinkers[i].isOff()) {
                blinkers[i].off();
            }
        }
    } while (duration < LED_COUNT * INIT_BLINK_OFFSET_MILLIS + INIT_BLINK_PERIOD_MILLIS);

    Serial.begin(115200);
}

class Parser {
public:
    void process(const uint8_t b) {
        switch (_state) {
        case STATE_EXPECT_COMMAND:
            _led = LED(b);
            if (_led < LED_COUNT) {
                switch (CMD(b)) {
                case CMD_ON:
                    blinkers[_led].on();
                    break;
                case CMD_OFF:
                    blinkers[_led].off();
                    break;
                case CMD_BLINK:
                    _state = STATE_EXPECT_BLINK_PERIOD_HIGH;
                    break;
                }
            }
            break;
        case STATE_EXPECT_BLINK_PERIOD_HIGH:
            _blink_period = b << 8;
            _state = STATE_EXPECT_BLINK_PERIOD_LOW;
            break;
        case STATE_EXPECT_BLINK_PERIOD_LOW:
            _blink_period |= b;
            if (_blink_period > 0) {
                blinkers[_led].blink(_blink_period);
            }
            _state = STATE_EXPECT_COMMAND;
            break;
        }
    }

private:
    enum {
        STATE_EXPECT_COMMAND,
        STATE_EXPECT_BLINK_PERIOD_HIGH,
        STATE_EXPECT_BLINK_PERIOD_LOW
    } _state = STATE_EXPECT_COMMAND;

    uint8_t _led;
    uint16_t _blink_period;
};

void loop() {
    static Parser parser;
    static int i = 0;

    int c;
    while ((c = Serial.read()) >= 0) {
        parser.process(c);
    }

    // minimize time between UART RX polls by updating one blinker and debouncer at a time
    blinkers[i].update();

    if (debouncers[i].update()) {
        if (debouncers[i].get()) {
            // button up (-> HIGH)
            Serial.write(CMD_OFF | i);
        } else {
            // button down (-> LOW)
            Serial.write(CMD_ON | i);
        }
    }

    i = (i + 1) % LED_COUNT;
}
