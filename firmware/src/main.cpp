#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MCP23017.h>
#include <Debouncer.h>

#include <Blinker.h>

#include "transform.h"

#define LED_COUNT 16

#if LED_COUNT >= 1 && LED_COUNT <= 16
const uint16_t ALL_LED_BITS = (1L << LED_COUNT) - 1;
#else
#error LED_COUNT must be a value from 1 to 16.
#endif

// configuration for initialization animation
const unsigned long long INIT_BLINK_OFFSET_MILLIS = 50;
const uint16_t INIT_BLINK_PERIOD_MILLIS = 500;

const char *LED_CHARS = "0123456789ABCDEF";

class PCA9685Blinker : public Blinker {
public:
    void attach(Adafruit_PWMServoDriver *pca9685, uint8_t pin) {
        _pca9685 = pca9685;
        _pin = pin;
        _prev = UINT16_MAX;
        // The PWM registers of PCA9685 may still hold their old values after an AVR reset or brief power-down.
        // Perform an initial update to turn them off. Otherwise there might be a short flicker once ~OE is pulled low.
        update();
    }

    void update() {
        Blinker::update(micros());
    }

protected:
    void render(float value) override {
        uint16_t cur = 4095 * Transform::gamma<9, 4>(value);
        if (cur != _prev) {
            _pca9685->setPin(_pin, cur);
            _prev = cur;
        }
    }

private:
    Adafruit_PWMServoDriver *_pca9685;
    uint8_t _pin;
    uint16_t _prev;
};

class MCP23017Debouncer : public Debouncer {
public:
    MCP23017Debouncer() : Debouncer(10), _mcp23017(nullptr), _pin(0) {
    }

    void attach(Adafruit_MCP23017 *mcp23017, uint8_t pin) {
        _mcp23017 = mcp23017;
        _pin = pin;
        update();
    }

    bool update() {
        return Debouncer::update(_mcp23017->digitalRead(_pin));
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
        blinkers[i].attach(&pca9685, i);
        blinkers[i].begin(micros());

        mcp23017.pinMode(i, INPUT);
        mcp23017.pullUp(i, HIGH);
        debouncers[i].attach(&mcp23017, i);
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
    bool process(const char ch) {
        if (_state == STATE_LED_SELECT) {
            const char *p = strchr(LED_CHARS, ch);
            if (p) {
                uint8_t tmp = p - LED_CHARS;
                if (tmp >= LED_COUNT) {
                    _reset();
                    return false;
                }
                _led = tmp;
                return true;
            }
            if (ch == '+') {
                blinkers[_led].on();
                _reset();
                return true;
            }
            if (ch == '-') {
                blinkers[_led].off();
                _reset();
                return true;
            }
            if (ch == '*') {
                _state = STATE_BLINK_PERIOD_SELECT;
                return true;
            }
            _reset();
            return false;
        }

        if (_state == STATE_BLINK_PERIOD_SELECT) {
            if (ch >= '0' && ch <= '9') {
                uint16_t tmp = 10 * _blink_period_millis + (ch - '0');
                if (tmp < _blink_period_millis) {
                    _reset();
                    return false;
                }
                _blink_period_millis = tmp;
                return true;
            }
            if (ch == '*') {
                blinkers[_led].blink(_blink_period_millis);
                _reset();
                return true;
            }
            _reset();
            return false;
        }

        _reset();
        return false;
    }

private:
    enum {
        STATE_LED_SELECT,
        STATE_BLINK_PERIOD_SELECT
    } _state = STATE_LED_SELECT;

    uint8_t _led = 0;
    uint16_t _blink_period_millis = 0;

    void _reset() {
        _state = STATE_LED_SELECT;
        _led = 0;
        _blink_period_millis = 0;
    }
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
            Serial.print(LED_CHARS[i]);
            Serial.print('-');
        } else {
            Serial.print(LED_CHARS[i]);
            Serial.print('+');
        }
    }

    i = (i + 1) % LED_COUNT;
}
