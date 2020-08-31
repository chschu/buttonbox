#ifndef DEBOUNCER_H
#define DEBOUNCER_H

#include <inttypes.h>

class Debouncer {
public:
    void begin(uint8_t initValue, uint32_t bounceTicks);

    // returns true if the stable value has changed
    bool update(uint8_t unstableValue, uint32_t currentTicks);

    // returns the stable value
    uint8_t get();
  
private:
  uint32_t _bounceTicks;

  uint8_t _stableValue;
  
  uint8_t _lastUnstableChangeValue;
  uint32_t _lastUnstableChangeTicks;
};

#endif


