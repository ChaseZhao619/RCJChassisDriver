#ifndef BSP_BE1732_H
#define BSP_BE1732_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 7 通道红外传感器协议常量。I2C 地址按 HAL 接口要求在实现中左移一位。 */
#define BSP_BE1732_I2C_ADDR                 0x01U
#define BSP_BE1732_CHANNEL_COUNT            7U
/* 命令 1~7 读取对应通道，其余命令读取统计值或切换/校准工作模式。 */
#define BSP_BE1732_CMD_CHANNEL_1            1U
#define BSP_BE1732_CMD_STRONGEST_CHANNEL    8U
#define BSP_BE1732_CMD_STRONGEST_VALUE      9U
#define BSP_BE1732_CMD_WEAKEST_CHANNEL      10U
#define BSP_BE1732_CMD_WEAKEST_VALUE        11U
#define BSP_BE1732_CMD_AVERAGE_VALUE        12U
#define BSP_BE1732_CMD_NORMAL_MODE          13U
#define BSP_BE1732_CMD_MODULATED_MODE       14U
#define BSP_BE1732_CMD_ZERO_CAL             15U

#ifndef BSP_BE1732_I2C_TIMEOUT_MS
/* 单次 I2C 操作超时 [ms]；增大可容忍慢响应，但会延长控制任务阻塞时间。 */
#define BSP_BE1732_I2C_TIMEOUT_MS           20U
#endif

#ifndef BSP_BE1732_I2C_READY_TRIALS
/* 初始化探测重试次数；过大只会延长设备缺失时的启动等待。 */
#define BSP_BE1732_I2C_READY_TRIALS         2U
#endif

#ifndef BSP_BE1732_NO_BALL_VALUE_THRESHOLD
/* 最强信号 <= 此值时计为一次无球。增大后更容易报告无球。 */
#define BSP_BE1732_NO_BALL_VALUE_THRESHOLD  4U
#endif

#ifndef BSP_BE1732_NO_BALL_COUNT_LIMIT
/* 连续无球样本确认次数。确认延迟=本值*FilteredChannel 调用周期。 */
#define BSP_BE1732_NO_BALL_COUNT_LIMIT      30U
#endif

typedef enum
{
    BSP_BE1732_MODE_NORMAL = 0,
    BSP_BE1732_MODE_MODULATED,
} BspBe1732Mode;

HAL_StatusTypeDef BspBe1732_Init(void);
HAL_StatusTypeDef BspBe1732_ReadCommand(uint8_t command, uint8_t *value);
HAL_StatusTypeDef BspBe1732_ReadChannelValue(uint8_t channel, uint8_t *value);
HAL_StatusTypeDef BspBe1732_ReadStrongestChannel(uint8_t *channel);
HAL_StatusTypeDef BspBe1732_ReadStrongestValue(uint8_t *value);
/* 返回 1~7 表示球方向，返回 -1 表示达到连续无球计数；期间保持上次有效通道。 */
HAL_StatusTypeDef BspBe1732_ReadFilteredChannel(int16_t *channel);
uint8_t BspBe1732_GetNoBallValueThreshold(void);
/* 设置并写入 Flash；不要在周期任务中反复调用，以免消耗 Flash 擦写寿命。 */
HAL_StatusTypeDef BspBe1732_SetNoBallValueThreshold(uint8_t threshold);
HAL_StatusTypeDef BspBe1732_SetMode(BspBe1732Mode mode);
BspBe1732Mode BspBe1732_GetMode(void);
uint32_t BspBe1732_GetLastI2cError(void);
uint32_t BspBe1732_GetLastFlashError(void);

#ifdef __cplusplus
}
#endif

#endif
