#ifndef DEBOUNCER_H
#define DEBOUNCER_H

#include <inttypes.h>

class Debouncer {
public:
    void begin(uint8_t initValue, uint16_t bounceMillis);
    // returns true if the stable value has changed
    // micros passed as parameter to avoid Arduino dependency
    bool update(uint8_t unstableValue, unsigned long micros);

    // returns the stable value
    uint8_t get();
  
private:
  unsigned long _bounceMicros;

  uint8_t _stableValue;
  
  uint8_t _lastUnstableChangeValue;
  unsigned long _lastUnstableChangeMicros;
};

#endif


