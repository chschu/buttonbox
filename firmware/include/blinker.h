#ifndef BLINKER_H
#define BLINKER_H

class Blinker {
public:
    virtual void on();
    virtual void off();
    virtual void blink(uint16_t periodMillis);

    virtual void begin() = 0;
    virtual void update() = 0;
};

class PolledCycleTimeout  {
public:
    PolledCycleTimeout(uint32_t timeoutCycles) {
        _timeoutCycles = timeoutCycles;
        _lastExpiredAtCycles = ESP.getCycleCount();
    }

    uint32_t expireCount() {
        uint32_t count = (ESP.getCycleCount() - _lastExpiredAtCycles) / _timeoutCycles;
        _lastExpiredAtCycles += count * _timeoutCycles;
        return count;
    }

private:
    uint32_t _lastExpiredAtCycles;
    uint32_t _timeoutCycles;
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
        _timeoutPtr.reset(new PolledCycleTimeout(1LL * F_CPU * periodMillis / 1000 / (2 * 255)));
    }

    void begin() {
        _cur_value = 0;
        _target_value = 0;
        _timeoutPtr.reset();

        Renderer::render(_cur_value);
    }

    void update() {
        if (!_timeoutPtr) {
            // no blink in progress; set target value immediately
            if (_cur_value != _target_value) {
                _cur_value = _target_value;
                Renderer::render(_cur_value);
            }
        } else {
            // blink in progress; check if there has been at least one expiration
            // if there is more than one expiration, the blink will be slower than desired
            if (_timeoutPtr->expireCount()) {
                // change direction if necessary
                if (_cur_value == 255) {
                    _delta = -1;
                } else if (_cur_value == 0) {
                    _delta = 1;
                }

                // update value and display
                _cur_value += _delta;
                Renderer::render(_cur_value);

                // stop blink at target value
                if (_stop_at_target_value && _cur_value == _target_value) {
                    _timeoutPtr.reset();
                }
            }
        }
    }

private:
    uint8_t _cur_value;

    int8_t _delta;

    bool _stop_at_target_value;
    uint8_t _target_value;

    std::unique_ptr<PolledCycleTimeout> _timeoutPtr;
};

#endif
