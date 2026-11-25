#ifndef UTIL_H_
#define UTIL_H_

#include <stdbool.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif





static inline uint8_t value_bit_set(uint16_t *target, int bit, bool val)
{
    if(val)
    {
        *target |= (1<<bit);
    }
    else {
        *target &= ~(1<<bit);
    }
    return 0;

}

static inline void value_bit_clear(uint16_t *target, int bit)
{
    *target &= ~(1<<bit);
}

static inline uint16_t le16_format_get(const uint8_t src[2])
{
    return ((uint16_t)src[1] << 8) | src[0];
}

static inline void le16_format_put(uint16_t val, uint8_t dst[2])
{
    dst[0] = val;
    dst[1] = val >> 8;
}

static inline void le32_format_put(uint32_t val, uint8_t dst[4])
{
    le16_format_put(val, dst);
    le16_format_put(val >> 16, &dst[2]);
}

static inline uint32_t le32_format_get(const uint8_t src[4])
{
    return ((uint32_t)le16_format_get(&src[2]) << 16) | le16_format_get(&src[0]);
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
