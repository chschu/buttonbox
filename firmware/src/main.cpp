#include <Arduino.h>

#include <memory>
#include <vector>

#include "blinker.h"
#include "renderer.h"

std::vector<std::shared_ptr<Blinker>> blinkers;

template <uint8_t channel, uint8_t bit_num>
//using DefaultLedcBlinker = DefaultBlinker<EaseSineOutRenderer<GammaRenderer<LedcWriteRenderer<channel, bit_num>>>>;
using DefaultLedcBlinker = DefaultBlinker<GammaRenderer<LedcWriteRenderer<channel, bit_num>>>;

void setup() {
    Serial.begin(9600);

    for (uint8_t ch = 0; ch < 8; ch++) {
        ledcSetup(ch, 1000, 15);
    }

    ledcAttachPin(4, 0);
    ledcAttachPin(16, 1); // RX2
    ledcAttachPin(17, 2); // TX2
    ledcAttachPin(18, 3);
    ledcAttachPin(19, 4);
    ledcAttachPin(21, 5);
    ledcAttachPin(22, 6);
    ledcAttachPin(23, 7);

    blinkers.push_back(std::make_shared<DefaultLedcBlinker<0, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<1, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<2, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<3, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<4, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<5, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<6, 15>>());
    blinkers.push_back(std::make_shared<DefaultLedcBlinker<7, 15>>());

    for (auto blinker : blinkers) {
        blinker->begin();
    }

    unsigned long millis0 = millis();
    int hasBeenOnMask = 0;
    int isOffMask;
    do {
        yield();

        int p = (millis() - millis0) / 100;
        if (p < blinkers.size()) {
            blinkers[p]->blink(500);
        }

        isOffMask = 0;
        int i = 0;
        for (auto blinker : blinkers) {
            blinker->update();
            if (blinker->isOff()) {
                isOffMask |= (1 << i);
            } else {
                hasBeenOnMask |= (1 << i);
                blinker->off();
            }
            i++;
        }
    } while (hasBeenOnMask != (1 << blinkers.size()) - 1 || isOffMask != hasBeenOnMask);
}

class Parser {
public:
    bool processToken(const String &token) {
        if (_state == STATE_INIT) {
            if (token == "led.") {
                _state = STATE_LED;
                return true;
            } else {
                _state = STATE_INIT;
                return false;
            }
        } else if (_state == STATE_LED) {
            unsigned int varN = 0;
            int i = 0;
            char c;
            while ((c = token[i++]) != '.') {
                if (c < '0' || c > '9') {
                    _state = STATE_INIT;
                    return false;
                }
                varN = 10 * varN + (c - '0');
                if (varN >= blinkers.size()) {
                    _state = STATE_INIT;
                    return false;
                }
            }
            _state = STATE_LED_N;
            _varN = varN;
            return true;
        } else if (_state == STATE_LED_N) {
            if (token == "on.") {
                blinkers[_varN]->on();
                _state = STATE_INIT;
                return true;
            } else if (token == "off.") {
                blinkers[_varN]->off();
                _state = STATE_INIT;
                return true;
            } else if (token == "blink.") {
                _state = STATE_LED_N_BLINK;
                return true;
            } else {
                _state = STATE_INIT;
                return false;
            }
        } else if (_state == STATE_LED_N_BLINK) {
            unsigned int periodMillis = 0;
            int i = 0;
            char c;
            while ((c = token[i++]) != '.') {
                if (c < '0' || c > '9') {
                    _state = STATE_INIT;
                    return false;
                }
                periodMillis = 10 * periodMillis + (c - '0');
                if (periodMillis > 65535) {
                    _state = STATE_INIT;
                    return false;
                }
            }
            if (periodMillis == 0) {
                _state = STATE_INIT;
                return false;
            }
            blinkers[_varN]->blink(periodMillis);
            _state = STATE_INIT;
            return true;
        } else {
            _state = STATE_INIT;
            return false;
        }
    }
private:
    enum {
        STATE_INIT,
        STATE_LED,
        STATE_LED_N,
        STATE_LED_N_BLINK,
    } _state = STATE_INIT;

    int _varN;
};

Parser parser;
String command;

void loop() {
    int c;
    while ((c = Serial.read()) >= 0) {
        command += (char) c;
        if (command.endsWith(".")) {
            parser.processToken(command);
            command.clear();
        }
    }

    for (auto blinker : blinkers) {
        blinker->update();
    }
}
