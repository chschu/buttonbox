#ifndef RENDERER_H
#define RENDERER_H

#include <esp32-hal-ledc.h>

template<uint8_t channel, uint8_t bit_num>
class LedcWriteRenderer {
public:
    static void render(float value) {
        ledcWrite(channel, value * ((1 << bit_num) - 1));
    }
};

template<typename DownstreamRenderer>
class EaseSineInRenderer {
public:
    static void render(float value) {
        DownstreamRenderer::render(1.0f - cosf(value * PI / 2.0f));
    }
};

template<typename DownstreamRenderer>
class EaseSineOutRenderer {
public:
    static void render(float value) {
        DownstreamRenderer::render(sinf(value * PI / 2.0f));
    }
};


template<typename DownstreamRenderer>
class EaseSineInOutRenderer {
public:
    static void render(float value) {
        DownstreamRenderer::render(0.5f - cosf(value * PI) / 2.0f);
    }
};

template<typename DownstreamRenderer>
class GammaRenderer {
public:
    static void render(float value) {
        DownstreamRenderer::render(powf(value, 2.2f));
    }
};

#endif
