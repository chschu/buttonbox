#include <assert.h>
#include <math.h>

#include "Blinker.h"

void Blinker::on() {
    stopAtPhase(0.5f);
}

void Blinker::off() {
    stopAtPhase(0.0f);
}

void Blinker::stopAtPhase(float phase) {
    assert(phase >= 0.0f && phase < 1.0f);
    _stopAtPhase = phase;
}

void Blinker::blink(uint16_t periodMillis) {
    assert(periodMillis > 0);
    _stopAtPhase = -1.0f;
    _blinking = true;
    _periodMicros = 1000L * periodMillis;
}

void Blinker::begin(unsigned long micros) {
    _phase = 0.0f;
    _stopAtPhase = -1.0f;
    _blinking = false;
    _phaseUpdatedAtMicros = micros;
    render(0.0f);
}

void Blinker::update(unsigned long micros) {
    unsigned long deltaMicros = micros - _phaseUpdatedAtMicros;
    _phaseUpdatedAtMicros += deltaMicros;

    float prev_phase = _phase;
    float unused;

    if (_stopAtPhase >= 0.0f && (!_blinking || deltaMicros >= modff(_stopAtPhase + 1.0f - _phase, &unused) * _periodMicros)) {
        _phase = _stopAtPhase;
        _stopAtPhase = -1.0f;
        _blinking = false;
    }

    if (_blinking) {
        _phase = modff(_phase + 1.0f * deltaMicros / _periodMicros, &unused);
    }

    if (_phase != prev_phase) {
        render(1.0f - fabsf(2.0f * _phase - 1.0f));
    }
}

bool Blinker::isBlinking() {
    return _blinking;
}

bool Blinker::isOff() {
    return _phase == 0.0f;
}
