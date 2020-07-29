#ifndef BLINKER_H
#define BLINKER_H

#include <inttypes.h>

class Blinker {
public:
    // micros passed as parameter to avoid Arduino dependency
    void begin(unsigned long micros);
    void update(unsigned long micros);

    void on();
    void off();
    void stopAtPhase(float phase);
    void blink(uint16_t periodMillis);

    bool isBlinking();

    bool isOff();

protected:
    virtual void render(float value) = 0;

private:
    unsigned long _phaseUpdatedAtMicros;
    float _phase; // [0.0, 1.0)

    unsigned long _periodMicros;

    float _stopAtPhase;
    bool _blinking;
};

#endif
