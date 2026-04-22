#ifndef BSP_CHASSIS_H
#define BSP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_CHASSIS_DIAMETER_MM                 210.0f
#define BSP_CHASSIS_CONTACT_OPPOSITE_MM         192.0f
#define BSP_CHASSIS_CONTACT_ADJACENT_MM         140.0f
#define BSP_CHASSIS_ROTATION_RADIUS_MM          (BSP_CHASSIS_CONTACT_OPPOSITE_MM * 0.5f)

#ifndef BSP_CHASSIS_DEFAULT_MAX_CURRENT
#define BSP_CHASSIS_DEFAULT_MAX_CURRENT         3000
#endif

#ifndef BSP_CHASSIS_MOTOR1_DIR
#define BSP_CHASSIS_MOTOR1_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR2_DIR
#define BSP_CHASSIS_MOTOR2_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR3_DIR
#define BSP_CHASSIS_MOTOR3_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR4_DIR
#define BSP_CHASSIS_MOTOR4_DIR                  1
#endif

typedef struct
{
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
    int16_t motor4;
} BspChassisMotorCurrent;

typedef struct
{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} BspChassisWheelDemand;

HAL_StatusTypeDef BspChassis_Stop(void);
HAL_StatusTypeDef BspChassis_SetMotorCurrents(int16_t motor1,
                                              int16_t motor2,
                                              int16_t motor3,
                                              int16_t motor4);
HAL_StatusTypeDef BspChassis_SetOpenLoop(int16_t forward_current,
                                         int16_t left_current,
                                         int16_t ccw_current);
HAL_StatusTypeDef BspChassis_SetOpenLoopLimited(int16_t forward_current,
                                                int16_t left_current,
                                                int16_t ccw_current,
                                                int16_t max_current);
HAL_StatusTypeDef BspChassis_SetVelocity(float vx_mm_s,
                                         float vy_mm_s,
                                         float wz_rad_s,
                                         int16_t max_current);
void BspChassis_CalcWheelDemand(float vx_mm_s,
                                float vy_mm_s,
                                float wz_rad_s,
                                BspChassisWheelDemand *demand);
const BspChassisMotorCurrent *BspChassis_GetLastCurrent(void);

#ifdef __cplusplus
}
#endif

#endif
