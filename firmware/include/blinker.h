#ifndef BLINKER_H
#define BLINKER_H

#include <PolledTimeout.h>

class Blinker {
public:
    virtual void on();
    virtual void off();
    virtual void blink(uint16_t periodMillis);

    virtual void begin() = 0;
    virtual void update() = 0;
};

template<typename Renderer>
class DefaultBlinker : public Blinker {
public:
    void on() {
        _target_value = 255;
        _stop_at_target_value = true;
    }

    void off() {
        _target_value = 0;
        _stop_at_target_value = true;
    }

    void blink(uint16_t periodMillis) {
        _stop_at_target_value = false;
        // maximum number of nanos per tick: 128500000
        _tick.reset(new esp8266::polledTimeout::periodicFastNs(1000000LL * periodMillis / (2 * 255)));
    }

    void begin() {
        _cur_value = 0;
        _target_value = 0;
        _tick.reset();

        Renderer::render(_cur_value);
    }

    void update() {
        if (!_tick) {
            // no blink in progress, set target value immediately
            if (_cur_value != _target_value) {
                _cur_value = _target_value;
                Renderer::render(_cur_value);
            }
        } else if (*_tick) {
            // blink in progress, polled timeout has expired

            // change direction if necessary
            if (_cur_value == 255) {
                _delta = -1;
            } else if (_cur_value == 0) {
                _delta = 1;
            }

            // update value
            _cur_value += _delta;
            Renderer::render(_cur_value);

            // stop blink at target value
            if (_stop_at_target_value && _cur_value == _target_value) {
                _tick.reset();
            }
        }
    }

private:
    uint8_t _cur_value;

    int8_t _delta;

    bool _stop_at_target_value;
    uint8_t _target_value;

    std::unique_ptr<esp8266::polledTimeout::periodicFastNs> _tick;
};

#endif
