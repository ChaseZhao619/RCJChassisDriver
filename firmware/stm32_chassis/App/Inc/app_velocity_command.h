#ifndef APP_VELOCITY_COMMAND_H
#define APP_VELOCITY_COMMAND_H

#include <stdint.h>

uint16_t AppVelocity_Crc16Ccitt(const uint8_t *data, uint16_t size);
uint8_t AppVelocity_Parse(const char *args, float *forward_mm_s,
                          float *left_mm_s, float *yaw_rad_s);

#endif
