#ifndef BLINKER_H
#define BLINKER_H

class Blinker {
public:
    virtual void on();
    virtual void off();
    virtual void blink(uint16_t periodMillis);

    virtual void begin() = 0;
    virtual void update() = 0;
    virtual bool isOff() = 0;
};

template<typename Renderer>
class DefaultBlinker : public Blinker {
public:
    void on() {
        _nextOn = true;
        _nextOff = false;
    }

    void off() {
        _nextOn = false;
        _nextOff = true;
    }

    void blink(uint16_t periodMillis) {
        assert(periodMillis > 0);
        _nextOn = false;
        _nextOff = false;
        _blinking = true;
        _periodMicros = 1000L * periodMillis;
    }

    void begin() {
        _nextOn = false;
        _nextOff = false;
        _blinking = false;
        _phase = 0.0;
        _phaseUpdatedAtMicros = micros();
    }

    void update() {
        unsigned long deltaMicros = micros() - _phaseUpdatedAtMicros;
        _phaseUpdatedAtMicros += deltaMicros;

        if (_nextOn) {
            if (!_blinking || deltaMicros >= fmodf(1.5f - _phase, 1.0f) * _periodMicros) {
                _blinking = false;
                _nextOn = false;
                _phase = 0.5f;
            }
        } else if (_nextOff) {
            if (!_blinking || deltaMicros >= fmodf(1.0f - _phase, 1.0f) * _periodMicros) {
                _blinking = false;
                _nextOff = false;
                _phase = 0.0f;
            }
        }

        if (_blinking) {
            _phase = fmodf(_phase + 1.0f * deltaMicros / _periodMicros, 1.0f);
        }

        Renderer::render(1.0f - fabsf(2.0f * _phase - 1.0f));
    }

    bool isOff() {
        return _phase == 0.0f;
    }

private:
    unsigned long _phaseUpdatedAtMicros;
    float _phase; // [0.0, 1.0)

    unsigned long _periodMicros;

    boolean _blinking;

    boolean _nextOn;
    boolean _nextOff;
};

#endif
