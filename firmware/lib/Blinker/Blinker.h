#ifndef BLINKER_H
#define BLINKER_H

#include <inttypes.h>

class Blinker {
public:
    void on();
    void off();
    void blink(uint16_t periodMillis);

    // micros passes as parameter to avoid Arduino dependency
    void begin(unsigned long micros);
    void update(unsigned long micros);

    bool isOff();

protected:
    virtual void render(float value) = 0;

private:
    unsigned long _phaseUpdatedAtMicros;
    float _phase; // [0.0, 1.0)

    unsigned long _periodMicros;

    uint8_t _next;
    bool _blinking;
};

#endif
