#include <assert.h>
#include <math.h>

#include "Blinker.h"

void Blinker::on() {
    _next_on = true;
    _next_off = false;
}

void Blinker::off() {
    _next_on = false;
    _next_off = true;
}

void Blinker::blink(uint16_t periodMillis) {
    assert(periodMillis > 0);
    _next_on = false;
    _next_off = false;
    _blinking = true;
    _periodMicros = 1000L * periodMillis;
}

void Blinker::begin(unsigned long micros) {
    _next_on = false;
    _next_off = false;
    _blinking = false;
    _phase = 0.0;
    _phaseUpdatedAtMicros = micros;
}

void Blinker::update(unsigned long micros) {
    unsigned long deltaMicros = micros - _phaseUpdatedAtMicros;
    _phaseUpdatedAtMicros += deltaMicros;

    float prev_phase = _phase;

    if (_next_on) {
        if (!_blinking || deltaMicros >= fmodf(1.5f - _phase, 1.0f) * _periodMicros) {
            _next_on = false;
            _blinking = false;
            _phase = 0.5f;
        }
    } else if (_next_off) {
        if (!_blinking || deltaMicros >= fmodf(1.0f - _phase, 1.0f) * _periodMicros) {
            _next_on = false;
            _blinking = false;
            _phase = 0.0f;
        }
    }

    if (_blinking) {
        _phase = fmodf(_phase + 1.0f * deltaMicros / _periodMicros, 1.0f);
    }

    if (_phase != prev_phase) {
        render(1.0f - fabsf(2.0f * _phase - 1.0f));
    }
}

bool Blinker::isOff() {
    return _phase == 0.0f;
}
