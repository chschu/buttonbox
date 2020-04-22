#include "Debouncer.h"

void Debouncer::begin(uint8_t initValue, uint16_t bounceMillis) {
    _lastUnstableChangeValue = _stableValue = initValue;
    _bounceMicros = 1000L * bounceMillis;
}

bool Debouncer::update(uint8_t unstableValue, unsigned long micros) {
    if (unstableValue != _lastUnstableChangeValue) {
        _lastUnstableChangeValue = unstableValue;
        _lastUnstableChangeMicros = micros;
        return false;
    }

    if (unstableValue != _stableValue && micros - _lastUnstableChangeMicros > _bounceMicros) {
        _stableValue = unstableValue;
        return true;
    }

    return false;
}

uint8_t Debouncer::get() {
   return _stableValue;
}
