/*
 * vector_utils.h
 *
 *  Created on: Jul 18, 2026
 *      Author: vietht-hl
 */

#ifndef UTILS_VECTOR_UTILS_H_
#define UTILS_VECTOR_UTILS_H_

#include <math.h>
#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3i16_t;


static inline Vector3f_t Vector3f_Zero(void)
{
    return (Vector3f_t){0.0f, 0.0f, 0.0f};
}

static inline Vector3f_t Vector3f_FromInt16(Vector3i16_t vector)
{
    return (Vector3f_t){
        (float)vector.x,
        (float)vector.y,
        (float)vector.z
    };
}

static inline Vector3f_t Vector3f_Subtract(Vector3f_t a, Vector3f_t b)
{
    return (Vector3f_t){
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}

/* Caller phải bảo đảm divisor khác 0. */
static inline Vector3f_t Vector3f_DivideScalar(
        Vector3f_t vector,
        float divisor)
{
    return (Vector3f_t){
        vector.x / divisor,
        vector.y / divisor,
        vector.z / divisor
    };
}

static inline float Vector3f_Magnitude(Vector3f_t vector)
{
    return sqrtf(
            vector.x * vector.x
          + vector.y * vector.y
          + vector.z * vector.z);
}

static inline void Vector3f_AccumulateInt16(
        Vector3f_t *accumulator,
        Vector3i16_t value)
{
    accumulator->x += (float)value.x;
    accumulator->y += (float)value.y;
    accumulator->z += (float)value.z;
}

static inline void Vector3i16_UpdateMinMax(
        Vector3i16_t *minimum,
        Vector3i16_t *maximum,
        Vector3i16_t sample)
{
    if (sample.x < minimum->x) minimum->x = sample.x;
    if (sample.y < minimum->y) minimum->y = sample.y;
    if (sample.z < minimum->z) minimum->z = sample.z;

    if (sample.x > maximum->x) maximum->x = sample.x;
    if (sample.y > maximum->y) maximum->y = sample.y;
    if (sample.z > maximum->z) maximum->z = sample.z;
}

static inline uint8_t Vector3i16_RangeExceeds(
        Vector3i16_t minimum,
        Vector3i16_t maximum,
        float threshold)
{
    const int32_t range_x = (int32_t)maximum.x - (int32_t)minimum.x;
    const int32_t range_y = (int32_t)maximum.y - (int32_t)minimum.y;
    const int32_t range_z = (int32_t)maximum.z - (int32_t)minimum.z;

    return (uint8_t)(
            (float)range_x > threshold
         || (float)range_y > threshold
         || (float)range_z > threshold);
}

#endif /* UTILS_VECTOR_UTILS_H_ */
