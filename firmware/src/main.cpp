#include <Arduino.h>

#include <memory>
#include <vector>

#include "blinker.h"
#include "renderer.h"

std::vector<std::shared_ptr<Blinker>> blinkers;

void setup() {
    Serial.begin(9600);

    analogWriteRange(255);

    blinkers.push_back(std::make_shared<DefaultBlinker<OnBoardLedRenderer<D4>>>());
    blinkers.push_back(std::make_shared<DefaultBlinker<OnBoardLedRenderer<D0>>>());

    for (auto blinker : blinkers) {
        blinker->begin();
    }
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
    int c = Serial.read();
    if (c >= 0) {
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
