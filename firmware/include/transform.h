#ifndef _TRANSFORM_H
#define _TRANSFORM_H

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

// powf() takes lots of flash space (~752 bytes on ATmega8)
// it can be avoided by using one of the template specializations
template<uint16_t n, uint16_t d>
float gamma(float value) {
    return powf(value, 1.0f * n / d);
}

template<>
float gamma<1, 1>(float value) {
    return value;
}

template<>
float gamma<5, 4>(float value) {
    return value * sqrtf(sqrtf(value));
}

template<>
float gamma<3, 2>(float value) {
    return value * sqrtf(value);
}

template<>
float gamma<7, 4>(float value) {
    return value * sqrtf(sqrtf(value * value * value));
}

template<>
float gamma<2, 1>(float value) {
    return value * value;
}

template<>
float gamma<9, 4>(float value) {
    return value * value * sqrtf(sqrtf(value));
}

template<>
float gamma<5, 2>(float value) {
    return value * value * sqrtf(value);
}

template<>
float gamma<11, 4>(float value) {
    return value * value * sqrtf(sqrtf(value * value * value));
}

template<>
float gamma<3, 1>(float value) {
    return value * value * value;
}

}

#endif