#include <Arduino.h>

#include <Blinker.h>

#include "transform.h"

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MCP23017.h>

#define LED_COUNT 16

#if LED_COUNT <= 8
#define led_mask_t uint8_t
#elif LED_COUNT <= 16
#define led_mask_t uint16_t
#else
#error LED_COUNT can be at most 16.
#endif

class PCA9685Blinker : public Blinker {
public:
    PCA9685Blinker(Adafruit_PWMServoDriver &pca9685, uint8_t pin)
        : _pca9685(pca9685), _pin(pin) {
    }
protected:
    void render(float value) override {
        uint16_t cur = 4095 * Transform::gamma3(value);
        if (cur != _prev) {
            _pca9685.setPin(_pin, cur);
            _prev = cur;
        }
    }
private:
    Adafruit_PWMServoDriver &_pca9685;
    uint8_t _pin;
    uint16_t _prev = UINT16_MAX;
};


Adafruit_PWMServoDriver pca9685;
Adafruit_MCP23017 mcp23017;
Blinker *blinkers[LED_COUNT];

void setup() {
    Serial.begin(115200);

    mcp23017.begin();
    pca9685.begin();

    // clock must be set after the above "begin" calls, because they set it to 100 kHz
    Wire.setClock(400000);

    // TODO output disable

    // configure open-drain outputs (default is totem-pole)
    pca9685.setOutputMode(false);

    // TODO Need to invert PCA9685 outputs? Set Bit 4 (INVRT) of Register 0x01 (MODE2) to 1

    // TODO output enable

    for (int i = 0; i < LED_COUNT; i++) {
        blinkers[i] = new PCA9685Blinker(pca9685, i);
    }

    mcp23017.setupInterrupts(true, false, LOW);
    for (int i = 0; i < LED_COUNT; i++) {
        mcp23017.pinMode(i, INPUT);
        mcp23017.pullUp(i, HIGH);
        mcp23017.setupInterruptPin(i, CHANGE);
    }

    for (auto blinker : blinkers) {
        blinker->begin(micros());
    }

    // perform initialization animation

    unsigned long micros0 = micros();
    led_mask_t hasBeenOnMask = 0;
    led_mask_t isOffMask;
    do {
        // TODO Remove yield call? It's probably not required on ATmega8.
        yield();

        int p = (micros() - micros0) / 100000;
        if (p < LED_COUNT) {
            blinkers[p]->blink(500);
        }

        isOffMask = 0;
        int i = 0;
        for (auto blinker : blinkers) {
            blinker->update(micros());
            if (blinker->isOff()) {
                isOffMask |= (1 << i);
            } else {
                hasBeenOnMask |= (1 << i);
                blinker->off();
            }
            i++;
        }
    } while (hasBeenOnMask != (1 << (LED_COUNT - 1) << 1) + 1 || isOffMask != hasBeenOnMask);
}

class Parser {
public:
    bool process(const char ch) {
        if (_state == STATE_LED_SELECT) {
            if (ch >= '0' && ch <= '9') {
                uint8_t tmp = 10 * _led + (ch - '0');
                 if (tmp < _led || tmp >= LED_COUNT) {
                    _reset();
                    return false;
                }
                _led = tmp;
                return true;
            }
            if (ch == '+') {
                blinkers[_led]->on();
                _reset();
                return true;
            }
            if (ch == '-') {
                blinkers[_led]->off();
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
                blinkers[_led]->blink(_blink_period_millis);
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
    static int blinker_index = 0;

    int c;
    while ((c = Serial.read()) >= 0) {
        parser.process(c);
    }

    // minimize time between UART RX polls by updating one blinker at a time
    blinkers[blinker_index]->update(micros());
    blinker_index = (blinker_index + 1) % LED_COUNT;

    uint8_t p = mcp23017.getLastInterruptPin();
    if (p != MCP23017_INT_ERR) {
        uint8_t v = mcp23017.getLastInterruptPinValue();
        if (v != MCP23017_INT_ERR) {
            Serial.print(p);
            Serial.print(v ? '-' : '+');
        }
    }
}
