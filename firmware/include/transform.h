#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <math.h>

namespace Transform {

float ease_sine_in(float value) {
    return 1.0f - cosf(value * M_PI / 2.0f);
}

float ease_sine_out(float value) {
    return sinf(value * M_PI / 2.0f);
}

float ease_sine_in_out(float value) {
    return 0.5f - cosf(value * M_PI) / 2.0f;
}

float gamma(float value, float gamma) {
    return pow(value, 2.2);
}

float gamma2(float value) {
    return value * value;
}

float gamma3(float value) {
    return value * value * value;
}

}

#endif