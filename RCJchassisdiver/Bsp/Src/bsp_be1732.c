#include "bsp_be1732.h"

#include "i2c.h"

static uint32_t be1732_last_i2c_error;
static uint8_t be1732_recovering;
static BspBe1732Mode be1732_mode = BSP_BE1732_MODE_MODULATED;

static uint16_t Be1732HalAddress(void)
{
    return (uint16_t)(BSP_BE1732_I2C_ADDR << 1);
}

static void Be1732Delay(void)
{
    volatile uint32_t delay;

    for (delay = 0U; delay < 600U; delay++)
    {
        __NOP();
    }
}

static void Be1732RecordI2cError(HAL_StatusTypeDef status)
{
    if (status != HAL_OK)
    {
        be1732_last_i2c_error = HAL_I2C_GetError(&hi2c2);
    }
}

static void Be1732RecoverBus(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    uint8_t pulse;

    if (be1732_recovering != 0U)
    {
        return;
    }
    be1732_recovering = 1U;

    (void)HAL_I2C_DeInit(&hi2c2);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    Be1732Delay();

    for (pulse = 0U; pulse < 9U; pulse++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        Be1732Delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        Be1732Delay();
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    Be1732Delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    Be1732Delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    Be1732Delay();

    (void)HAL_I2C_Init(&hi2c2);
    be1732_recovering = 0U;
}

HAL_StatusTypeDef BspBe1732_Init(void)
{
    HAL_StatusTypeDef status;

    be1732_last_i2c_error = HAL_I2C_ERROR_NONE;
    Be1732RecoverBus();
    status = HAL_I2C_IsDeviceReady(&hi2c2,
                                   Be1732HalAddress(),
                                   BSP_BE1732_I2C_READY_TRIALS,
                                   BSP_BE1732_I2C_TIMEOUT_MS);
    Be1732RecordI2cError(status);
    if (status == HAL_OK)
    {
        status = BspBe1732_SetMode(BSP_BE1732_MODE_MODULATED);
    }

    return status;
}

HAL_StatusTypeDef BspBe1732_ReadCommand(uint8_t command, uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t command_byte = command;

    if ((value == NULL) || (command == 0U) || (command > BSP_BE1732_CMD_ZERO_CAL))
    {
        return HAL_ERROR;
    }

    be1732_last_i2c_error = HAL_I2C_ERROR_NONE;

    if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET)
    {
        Be1732RecoverBus();
    }

    status = HAL_I2C_Mem_Read(&hi2c2,
                              Be1732HalAddress(),
                              command,
                              I2C_MEMADD_SIZE_8BIT,
                              value,
                              1U,
                              BSP_BE1732_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        return HAL_OK;
    }
    Be1732RecordI2cError(status);

    if (status == HAL_BUSY)
    {
        Be1732RecoverBus();
    }

    status = HAL_I2C_Master_Transmit(&hi2c2,
                                     Be1732HalAddress(),
                                     &command_byte,
                                     1U,
                                     BSP_BE1732_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        HAL_Delay(1U);
        status = HAL_I2C_Master_Receive(&hi2c2,
                                        Be1732HalAddress(),
                                        value,
                                        1U,
                                        BSP_BE1732_I2C_TIMEOUT_MS);
        if (status == HAL_OK)
        {
            be1732_last_i2c_error = HAL_I2C_ERROR_NONE;
            return HAL_OK;
        }
    }
    Be1732RecordI2cError(status);

    if (status == HAL_BUSY)
    {
        Be1732RecoverBus();
    }

    status = HAL_I2C_Master_Receive(&hi2c2,
                                    Be1732HalAddress(),
                                    value,
                                    1U,
                                    BSP_BE1732_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        be1732_last_i2c_error = HAL_I2C_ERROR_NONE;
        return HAL_OK;
    }
    Be1732RecordI2cError(status);

    return status;
}

HAL_StatusTypeDef BspBe1732_ReadChannelValue(uint8_t channel, uint8_t *value)
{
    if ((channel == 0U) || (channel > BSP_BE1732_CHANNEL_COUNT))
    {
        return HAL_ERROR;
    }

    return BspBe1732_ReadCommand((uint8_t)(BSP_BE1732_CMD_CHANNEL_1 + channel - 1U), value);
}

HAL_StatusTypeDef BspBe1732_ReadStrongestChannel(uint8_t *channel)
{
    HAL_StatusTypeDef status;
    uint8_t value;
    uint8_t ch;
    uint8_t channel_value;
    uint8_t max_channel = 0U;
    uint8_t max_value = 0U;

    if (channel == NULL)
    {
        return HAL_ERROR;
    }

    status = BspBe1732_ReadCommand(BSP_BE1732_CMD_STRONGEST_CHANNEL, &value);
    if (status != HAL_OK)
    {
        return status;
    }

    if ((value == 0U) || (value > BSP_BE1732_CHANNEL_COUNT))
    {
        for (ch = 1U; ch <= BSP_BE1732_CHANNEL_COUNT; ch++)
        {
            status = BspBe1732_ReadChannelValue(ch, &channel_value);
            if (status != HAL_OK)
            {
                return status;
            }

            if ((max_channel == 0U) || (channel_value > max_value))
            {
                max_channel = ch;
                max_value = channel_value;
            }
        }

        if (max_channel == 0U)
        {
            return HAL_ERROR;
        }

        *channel = max_channel;
        return HAL_OK;
    }

    *channel = value;
    return HAL_OK;
}

HAL_StatusTypeDef BspBe1732_SetMode(BspBe1732Mode mode)
{
    HAL_StatusTypeDef status;
    uint8_t ignored_value;
    uint8_t command;

    if (mode == BSP_BE1732_MODE_NORMAL)
    {
        command = BSP_BE1732_CMD_NORMAL_MODE;
    }
    else if (mode == BSP_BE1732_MODE_MODULATED)
    {
        command = BSP_BE1732_CMD_MODULATED_MODE;
    }
    else
    {
        return HAL_ERROR;
    }

    status = BspBe1732_ReadCommand(command, &ignored_value);
    if (status == HAL_OK)
    {
        be1732_mode = mode;
    }

    return status;
}

BspBe1732Mode BspBe1732_GetMode(void)
{
    return be1732_mode;
}

uint32_t BspBe1732_GetLastI2cError(void)
{
    return be1732_last_i2c_error;
}
