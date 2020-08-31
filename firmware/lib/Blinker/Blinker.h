#ifndef BLINKER_H
#define BLINKER_H

#include <inttypes.h>

class Blinker {
public:
    void begin(uint32_t currentTicks);
    void update(uint32_t currentTicks);

    void on();
    void off();
    void stopAtPhase(float phase);
    void blink(uint32_t periodTicks);

    bool isBlinking();

    bool isOff();

protected:
    virtual void render(float value) = 0;

private:
    uint32_t _phaseUpdatedAtTicks;
    float _phase; // [0.0, 1.0)

    uint32_t _periodTicks;

    float _stopAtPhase;
    bool _blinking;
};

#endif
