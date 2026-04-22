#include "bsp_motor.h"

#include "can.h"
#include <string.h>

static BspMotorFeedback motor_feedback[BSP_MOTOR_COUNT];
static int16_t motor_current[BSP_MOTOR_COUNT];
static CAN_HandleTypeDef *motor_can = &hcan1;

static int16_t LimitCurrent(int16_t current)
{
    if (current > BSP_MOTOR_C610_MAX_CURRENT)
    {
        return BSP_MOTOR_C610_MAX_CURRENT;
    }

    if (current < -BSP_MOTOR_C610_MAX_CURRENT)
    {
        return -BSP_MOTOR_C610_MAX_CURRENT;
    }

    return current;
}

static int8_t GetIndexFromCanId(uint8_t can_id)
{
    if ((can_id < 1U) || (can_id > BSP_MOTOR_COUNT))
    {
        return -1;
    }

    return (int8_t)(can_id - 1U);
}

static void PackCurrent(uint8_t *data, uint8_t offset, int16_t current)
{
    current = LimitCurrent(current);
    data[offset] = (uint8_t)((uint16_t)current >> 8);
    data[offset + 1U] = (uint8_t)((uint16_t)current & 0xFFU);
}

static HAL_StatusTypeDef SendCurrentFrame(uint32_t std_id,
                                          int16_t current1,
                                          int16_t current2,
                                          int16_t current3,
                                          int16_t current4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};
    uint32_t tx_mailbox;

    if (motor_can == NULL)
    {
        return HAL_ERROR;
    }

    tx_header.StdId = std_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    PackCurrent(data, 0U, current1);
    PackCurrent(data, 2U, current2);
    PackCurrent(data, 4U, current3);
    PackCurrent(data, 6U, current4);

    return HAL_CAN_AddTxMessage(motor_can, &tx_header, data, &tx_mailbox);
}

static void DecodeMotorFeedback(uint8_t can_id, const uint8_t data[8])
{
    int8_t index = GetIndexFromCanId(can_id);
    BspMotorFeedback *feedback;
    int32_t delta_ecd;

    if (index < 0)
    {
        return;
    }

    feedback = &motor_feedback[index];
    feedback->can_id = can_id;
    feedback->last_ecd = feedback->ecd;
    feedback->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    feedback->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    feedback->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    feedback->temperature = data[6];

    if (feedback->online != 0U)
    {
        delta_ecd = (int32_t)feedback->ecd - (int32_t)feedback->last_ecd;
        if (delta_ecd > (BSP_MOTOR_ENCODER_RANGE / 2))
        {
            feedback->round_count--;
        }
        else if (delta_ecd < -(BSP_MOTOR_ENCODER_RANGE / 2))
        {
            feedback->round_count++;
        }
    }

    feedback->total_ecd = (feedback->round_count * BSP_MOTOR_ENCODER_RANGE) + feedback->ecd;
    feedback->update_tick = HAL_GetTick();
    feedback->online = 1U;
}

HAL_StatusTypeDef BspMotor_Start(void)
{
    if (motor_can == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(motor_can) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_CAN_ActivateNotification(motor_can,
                                        CAN_IT_RX_FIFO0_MSG_PENDING |
                                        CAN_IT_BUSOFF |
                                        CAN_IT_ERROR);
}

HAL_StatusTypeDef BspMotor_Init(void)
{
    CAN_FilterTypeDef can_filter;

    motor_can = &hcan1;
    BspMotor_ResetFeedback();

    can_filter.FilterBank = 0U;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000U;
    can_filter.FilterIdLow = 0x0000U;
    can_filter.FilterMaskIdHigh = 0x0000U;
    can_filter.FilterMaskIdLow = 0x0000U;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(motor_can, &can_filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return BspMotor_Start();
}

HAL_StatusTypeDef BspMotor_Stop(void)
{
    if (motor_can == NULL)
    {
        return HAL_ERROR;
    }

    (void)BspMotor_SetCurrents(0, 0, 0, 0, 0);
    (void)HAL_CAN_DeactivateNotification(motor_can, CAN_IT_RX_FIFO0_MSG_PENDING);

    return HAL_CAN_Stop(motor_can);
}

HAL_StatusTypeDef BspMotor_SetCurrent(uint8_t can_id, int16_t current)
{
    int8_t index = GetIndexFromCanId(can_id);

    if (index < 0)
    {
        return HAL_ERROR;
    }

    motor_current[index] = LimitCurrent(current);

    if (can_id <= BSP_MOTOR_CHASSIS_COUNT)
    {
        return BspMotor_SendChassisCurrents(motor_current[0],
                                            motor_current[1],
                                            motor_current[2],
                                            motor_current[3]);
    }

    return BspMotor_SendFunctionCurrent(motor_current[4]);
}

HAL_StatusTypeDef BspMotor_SetCurrents(int16_t motor1,
                                       int16_t motor2,
                                       int16_t motor3,
                                       int16_t motor4,
                                       int16_t motor5)
{
    HAL_StatusTypeDef chassis_status;
    HAL_StatusTypeDef function_status;

    chassis_status = BspMotor_SendChassisCurrents(motor1, motor2, motor3, motor4);
    function_status = BspMotor_SendFunctionCurrent(motor5);

    if ((chassis_status != HAL_OK) || (function_status != HAL_OK))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BspMotor_SendChassisCurrents(int16_t motor1,
                                               int16_t motor2,
                                               int16_t motor3,
                                               int16_t motor4)
{
    motor_current[0] = LimitCurrent(motor1);
    motor_current[1] = LimitCurrent(motor2);
    motor_current[2] = LimitCurrent(motor3);
    motor_current[3] = LimitCurrent(motor4);

    return SendCurrentFrame(BSP_MOTOR_CAN_TX_ID_1_TO_4, motor1, motor2, motor3, motor4);
}

HAL_StatusTypeDef BspMotor_SendFunctionCurrent(int16_t motor5)
{
    motor_current[4] = LimitCurrent(motor5);

    return SendCurrentFrame(BSP_MOTOR_CAN_TX_ID_5_TO_8, motor5, 0, 0, 0);
}

const BspMotorFeedback *BspMotor_GetFeedback(uint8_t can_id)
{
    int8_t index = GetIndexFromCanId(can_id);

    if (index < 0)
    {
        return NULL;
    }

    return &motor_feedback[index];
}

const BspMotorFeedback *BspMotor_GetFeedbackByIndex(BspMotorIndex index)
{
    if ((uint32_t)index >= BSP_MOTOR_COUNT)
    {
        return NULL;
    }

    return &motor_feedback[index];
}

uint8_t BspMotor_IsOnline(uint8_t can_id, uint32_t timeout_ms)
{
    const BspMotorFeedback *feedback = BspMotor_GetFeedback(can_id);

    if ((feedback == NULL) || (feedback->online == 0U))
    {
        return 0U;
    }

    return ((HAL_GetTick() - feedback->update_tick) <= timeout_ms) ? 1U : 0U;
}

void BspMotor_ResetFeedback(void)
{
    uint8_t i;

    memset(motor_feedback, 0, sizeof(motor_feedback));
    memset(motor_current, 0, sizeof(motor_current));
    for (i = 0U; i < BSP_MOTOR_COUNT; i++)
    {
        motor_feedback[i].can_id = (uint8_t)(i + 1U);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    uint8_t can_id;

    if ((motor_can == NULL) || (hcan != motor_can))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
        {
            return;
        }

        if ((rx_header.IDE != CAN_ID_STD) ||
            (rx_header.RTR != CAN_RTR_DATA) ||
            (rx_header.DLC < 7U) ||
            (rx_header.StdId <= BSP_MOTOR_CAN_RX_BASE_ID))
        {
            continue;
        }

        can_id = (uint8_t)(rx_header.StdId - BSP_MOTOR_CAN_RX_BASE_ID);
        DecodeMotorFeedback(can_id, data);
    }
}
