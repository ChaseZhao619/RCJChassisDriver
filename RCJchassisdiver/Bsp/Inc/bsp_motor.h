#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_MOTOR_COUNT             5U
#define BSP_MOTOR_CHASSIS_COUNT     4U
#define BSP_MOTOR_CAN_RX_BASE_ID    0x200U
#define BSP_MOTOR_CAN_TX_ID_1_TO_4  0x200U
#define BSP_MOTOR_CAN_TX_ID_5_TO_8  0x1FFU
#define BSP_MOTOR_C610_MAX_CURRENT  10000
#define BSP_MOTOR_ENCODER_RANGE     8192

typedef enum
{
    BSP_MOTOR_CHASSIS_1 = 0,
    BSP_MOTOR_CHASSIS_2,
    BSP_MOTOR_CHASSIS_3,
    BSP_MOTOR_CHASSIS_4,
    BSP_MOTOR_FUNCTION,
} BspMotorIndex;

typedef struct
{
    uint8_t can_id;
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    int32_t round_count;
    int32_t total_ecd;
    uint32_t update_tick;
    uint8_t online;
} BspMotorFeedback;

HAL_StatusTypeDef BspMotor_Init(void);
HAL_StatusTypeDef BspMotor_Start(void);
HAL_StatusTypeDef BspMotor_Stop(void);
HAL_StatusTypeDef BspMotor_SetCurrent(uint8_t can_id, int16_t current);
HAL_StatusTypeDef BspMotor_SetCurrents(int16_t motor1,
                                       int16_t motor2,
                                       int16_t motor3,
                                       int16_t motor4,
                                       int16_t motor5);
HAL_StatusTypeDef BspMotor_SendChassisCurrents(int16_t motor1,
                                               int16_t motor2,
                                               int16_t motor3,
                                               int16_t motor4);
HAL_StatusTypeDef BspMotor_SendFunctionCurrent(int16_t motor5);
const BspMotorFeedback *BspMotor_GetFeedback(uint8_t can_id);
const BspMotorFeedback *BspMotor_GetFeedbackByIndex(BspMotorIndex index);
uint8_t BspMotor_IsOnline(uint8_t can_id, uint32_t timeout_ms);
void BspMotor_ResetFeedback(void);

#ifdef __cplusplus
}
#endif

#endif
