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

void Blinker::blink(uint32_t periodTicks) {
    assert(periodTicks > 0);
    _stopAtPhase = -1.0f;
    _blinking = true;
    _periodTicks = periodTicks;
}

void Blinker::begin(uint32_t currentTicks) {
    _phase = 0.0f;
    _stopAtPhase = -1.0f;
    _blinking = false;
    _phaseUpdatedAtTicks = currentTicks;
    render(0.0f);
}

void Blinker::update(uint32_t currentTicks) {
    uint32_t deltaTicks = currentTicks - _phaseUpdatedAtTicks;
    _phaseUpdatedAtTicks += deltaTicks;

    float prev_phase = _phase;
    float unused;

    if (_stopAtPhase >= 0.0f && (!_blinking || deltaTicks >= modff(_stopAtPhase + 1.0f - _phase, &unused) * _periodTicks)) {
        _phase = _stopAtPhase;
        _stopAtPhase = -1.0f;
        _blinking = false;
    }

    if (_blinking) {
        _phase = modff(_phase + 1.0f * deltaTicks / _periodTicks, &unused);
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
