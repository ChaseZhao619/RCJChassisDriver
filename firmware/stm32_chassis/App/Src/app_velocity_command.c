#include "app_velocity_command.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

uint16_t AppVelocity_Crc16Ccitt(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    if (data == 0) return crc;
    for (i = 0U; i < size; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint8_t AppVelocity_Parse(const char *args, float *forward_mm_s,
                          float *left_mm_s, float *yaw_rad_s)
{
    float *values[3] = {forward_mm_s, left_mm_s, yaw_rad_s};
    const char *cursor = args;
    char *end;
    int i;
    if ((args == 0) || (forward_mm_s == 0) || (left_mm_s == 0) || (yaw_rad_s == 0))
        return 0U;
    for (i = 0; i < 3; i++) {
        if (!isspace((unsigned char)*cursor)) return 0U;
        while (isspace((unsigned char)*cursor)) cursor++;
        *values[i] = strtof(cursor, &end);
        if ((end == cursor) || !isfinite(*values[i])) return 0U;
        cursor = end;
    }
    while (isspace((unsigned char)*cursor)) cursor++;
    if (*cursor != '\0') return 0U;
    return ((hypotf(*forward_mm_s, *left_mm_s) <= 650.0f) &&
            (fabsf(*yaw_rad_s) <= 2.0f)) ? 1U : 0U;
}
