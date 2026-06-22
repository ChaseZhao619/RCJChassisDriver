#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 当前管理 4 个底盘电机和 1 个功能电机。修改数量时需同步检查 CAN 打包逻辑。 */
#define BSP_MOTOR_COUNT             5U
#define BSP_MOTOR_CHASSIS_COUNT     4U
/* 大疆电调反馈帧 ID = 0x200 + CAN ID；发送帧分别控制 ID 1~4 和 5~8。 */
#define BSP_MOTOR_CAN_RX_BASE_ID    0x200U
#define BSP_MOTOR_CAN_TX_ID_1_TO_4  0x200U
#define BSP_MOTOR_CAN_TX_ID_5_TO_8  0x1FFU
/* C610 协议允许的控制量绝对值和单圈编码器计数，属于协议常量而非调参项。 */
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
    uint8_t can_id;          /* 电调 CAN ID，范围 1~5。 */
    uint16_t ecd;            /* 当前单圈机械角编码值，范围 0~8191。 */
    uint16_t last_ecd;       /* 上一帧编码值，用于跨零点圈数判断。 */
    int16_t speed_rpm;       /* 电调反馈的电机轴转速 [rpm]。 */
    int16_t given_current;   /* 电调反馈的实际/给定电流字段，量纲依电调协议。 */
    uint8_t temperature;     /* 电调反馈温度 [degC]。 */
    int32_t round_count;     /* 软件累计圈数；断电或 ResetFeedback 后清零。 */
    int32_t total_ecd;       /* 多圈累计编码值 = 圈数*8192 + 当前编码值。 */
    uint32_t update_tick;    /* 最近有效反馈的 HAL 毫秒时刻。 */
    uint8_t online;          /* 收到过有效反馈后置 1；实时超时应调用 IsOnline 判断。 */
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
