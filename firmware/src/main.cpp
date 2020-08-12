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

// 0x0N: Button N activated (on release)
const uint8_t CMD_BUTTON = 0x00;

// 0x0N: LED N off
const uint8_t CMD_OFF = 0x00;

// 0x1N: LED N on
const uint8_t CMD_ON = 0x10;

#if LED_COUNT >= 1 && LED_COUNT <= 16
const uint16_t ALL_LED_BITS = (1L << LED_COUNT) - 1;
#else
#error LED_COUNT must be a value from 1 to 16.
#endif

// configuration for initialization animation
const unsigned long long INIT_BLINK_OFFSET_MILLIS = 50;
const uint16_t INIT_BLINK_PERIOD_MILLIS = 500;

// configure blinking
const float BRIGHT_PHASE = 0.5f;
const float DARK_PHASE = 0.9f;
const uint16_t BLINK_PERIOD_MILLIS = 500;

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

PCA9685Blinker blinkers[LED_COUNT];
Debouncer debouncers[LED_COUNT];

void setup() {
    // initialize MCP23017 and PCA9685
    mcp23017.begin();
    pca9685.begin();

    // clock must be set after the above "begin" calls, because they set it to 100 kHz
    Wire.setClock(800000);

    // initialize output blinkers
    for (int i = 0; i < LED_COUNT; i++) {
        blinkers[i].begin(&pca9685, i);

        mcp23017.pinMode(i, INPUT);
        mcp23017.pullUp(i, HIGH);
    }

    // initialize input debouncers
    uint16_t inputs = mcp23017.readGPIOAB();
    for (int i = 0; i < LED_COUNT; i++) {
        debouncers[i].begin((inputs >> i) & 1, BOUNCE_MILLIS);
    }

    // pull ~OE low to enable LEDs
    DDRC |= (1 << DDC2);
    PORTC &= ~(1 << PORTC2);

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
                blinkers[i].stopAtPhase(DARK_PHASE);
            }
        }
    } while (duration < LED_COUNT * INIT_BLINK_OFFSET_MILLIS + INIT_BLINK_PERIOD_MILLIS);

    Serial.begin(115200);
}

void loop() {
    int c;
    while ((c = Serial.read()) >= 0) {
        switch (CMD(c)) {
        case CMD_OFF:
            blinkers[LED(c)].stopAtPhase(DARK_PHASE);
            break;
        case CMD_ON:
            blinkers[LED(c)].stopAtPhase(BRIGHT_PHASE);
            break;
        }
    }

    uint16_t inputs = mcp23017.readGPIOAB();
    for (int i = 0; i < LED_COUNT; i++) {
        blinkers[i].update();
        if (debouncers[i].update((inputs >> i) & 1, micros()) && !blinkers[i].isBlinking() && debouncers[i].get()) {
            blinkers[i].blink(BLINK_PERIOD_MILLIS);
            Serial.write(CMD_BUTTON | i);
        }
    }
}
