#include "Debouncer.h"

void Debouncer::begin(uint8_t initValue, uint32_t bounceTicks) {
    _lastUnstableChangeValue = _stableValue = initValue;
    _bounceTicks = bounceTicks;
}

bool Debouncer::update(uint8_t unstableValue, uint32_t currentTicks) {
    if (unstableValue != _lastUnstableChangeValue) {
        _lastUnstableChangeValue = unstableValue;
        _lastUnstableChangeTicks = currentTicks;
        return false;
    }

    if (unstableValue != _stableValue && currentTicks - _lastUnstableChangeTicks > _bounceTicks) {
        _stableValue = unstableValue;
        return true;
    }

    return false;
}

uint8_t Debouncer::get() {
   return _stableValue;
}
