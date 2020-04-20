#include <assert.h>
#include <math.h>

#include "Blinker.h"

static const uint8_t _NEXT_KEEP = 0;
static const uint8_t _NEXT_ON = 1;
static const uint8_t _NEXT_OFF = 2;

void Blinker::on() {
    _next = _NEXT_ON;
}

void Blinker::off() {
    _next = _NEXT_OFF;
}

void Blinker::blink(uint16_t periodMillis) {
    assert(periodMillis > 0);
    _next = _NEXT_KEEP;
    _blinking = true;
    _periodMicros = 1000L * periodMillis;
}

void Blinker::begin(unsigned long micros) {
    _next = _NEXT_KEEP;
    _blinking = false;
    _phase = 0.0f;
    _phaseUpdatedAtMicros = micros;
    render(0.0f);
}

void Blinker::update(unsigned long micros) {
    unsigned long deltaMicros = micros - _phaseUpdatedAtMicros;
    _phaseUpdatedAtMicros += deltaMicros;

    float prev_phase = _phase;
    float unused;

    if (_next != _NEXT_KEEP) {
        float target_phase = _next == _NEXT_ON ? 0.5f : 0.0f;
        if (!_blinking || deltaMicros >= modff(target_phase + 1.0f - _phase, &unused) * _periodMicros) {
            _next = _NEXT_KEEP;
            _blinking = false;
            _phase = target_phase;
        }
    }

    if (_blinking) {
        _phase = modff(_phase + 1.0f * deltaMicros / _periodMicros, &unused);
    }

    if (_phase != prev_phase) {
        render(1.0f - fabsf(2.0f * _phase - 1.0f));
    }
}

bool Blinker::isOff() {
    return _phase == 0.0f;
}
