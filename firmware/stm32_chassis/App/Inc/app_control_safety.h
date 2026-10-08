#ifndef APP_CONTROL_SAFETY_H
#define APP_CONTROL_SAFETY_H

#include <stdint.h>

/* Unsigned subtraction remains valid across HAL_GetTick() rollover. */
static inline uint8_t AppControlSafety_IsFresh(uint32_t now,
                                              uint32_t last_tick,
                                              uint32_t timeout_ms)
{
    return ((now - last_tick) <= timeout_ms) ? 1U : 0U;
}

#endif
