#ifndef SIM_PORT_ARM_MATH_H
#define SIM_PORT_ARM_MATH_H

#include <math.h>
#include <stdint.h>

static inline float arm_sin_f32(float x)
{
    return sinf(x);
}

static inline float arm_cos_f32(float x)
{
    return cosf(x);
}

static inline void arm_sqrt_f32(float in, float *out)
{
    *out = sqrtf(in);
}

#endif

