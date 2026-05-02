#include "bsp_be1732.h"

#include "i2c.h"

static uint32_t be1732_last_i2c_error;
static uint32_t be1732_last_flash_error;
static uint8_t be1732_recovering;
static BspBe1732Mode be1732_mode = BSP_BE1732_MODE_MODULATED;
static uint8_t be1732_no_ball_count;
static uint8_t be1732_last_valid_channel;
static uint8_t be1732_no_ball_value_threshold = BSP_BE1732_NO_BALL_VALUE_THRESHOLD;
static uint8_t be1732_param_loaded_valid;

#define BSP_BE1732_PARAM_FLASH_ADDR          0x08060000UL
#define BSP_BE1732_PARAM_FLASH_SECTOR        FLASH_SECTOR_7
#define BSP_BE1732_PARAM_MAGIC               0xBE1732A5UL
#define BSP_BE1732_PARAM_VERSION             1UL
#define BSP_BE1732_PARAM_CRC_XOR             0xA55A5AA5UL

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t no_ball_value_threshold;
    uint32_t crc;
} BspBe1732ParamRecord;

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

static uint32_t Be1732ParamCrc(const BspBe1732ParamRecord *record)
{
    if (record == NULL)
    {
        return 0UL;
    }

    return record->magic ^
           record->version ^
           record->no_ball_value_threshold ^
           BSP_BE1732_PARAM_CRC_XOR;
}

static void Be1732LoadParam(void)
{
    const BspBe1732ParamRecord *record =
        (const BspBe1732ParamRecord *)BSP_BE1732_PARAM_FLASH_ADDR;

    be1732_no_ball_value_threshold = BSP_BE1732_NO_BALL_VALUE_THRESHOLD;
    be1732_param_loaded_valid = 0U;

    if ((record->magic == BSP_BE1732_PARAM_MAGIC) &&
        (record->version == BSP_BE1732_PARAM_VERSION) &&
        (record->crc == Be1732ParamCrc(record)) &&
        (record->no_ball_value_threshold <= UINT8_MAX))
    {
        be1732_no_ball_value_threshold = (uint8_t)record->no_ball_value_threshold;
        be1732_param_loaded_valid = 1U;
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
    be1732_last_flash_error = HAL_FLASH_ERROR_NONE;
    Be1732LoadParam();
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

HAL_StatusTypeDef BspBe1732_ReadStrongestValue(uint8_t *value)
{
    if (value == NULL)
    {
        return HAL_ERROR;
    }

    return BspBe1732_ReadCommand(BSP_BE1732_CMD_STRONGEST_VALUE, value);
}

HAL_StatusTypeDef BspBe1732_ReadFilteredChannel(int16_t *channel)
{
    HAL_StatusTypeDef status;
    uint8_t raw_channel;
    uint8_t strongest_value;

    if (channel == NULL)
    {
        return HAL_ERROR;
    }

    status = BspBe1732_ReadStrongestChannel(&raw_channel);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BspBe1732_ReadStrongestValue(&strongest_value);
    if (status != HAL_OK)
    {
        return status;
    }

    if (strongest_value <= be1732_no_ball_value_threshold)
    {
        if (be1732_no_ball_count < UINT8_MAX)
        {
            be1732_no_ball_count++;
        }

        if (be1732_no_ball_count > BSP_BE1732_NO_BALL_COUNT_LIMIT)
        {
            *channel = -1;
            return HAL_OK;
        }

        if (be1732_last_valid_channel != 0U)
        {
            *channel = (int16_t)be1732_last_valid_channel;
            return HAL_OK;
        }
    }
    else
    {
        be1732_no_ball_count = 0U;
        be1732_last_valid_channel = raw_channel;
    }

    *channel = (int16_t)raw_channel;
    return HAL_OK;
}

uint8_t BspBe1732_GetNoBallValueThreshold(void)
{
    return be1732_no_ball_value_threshold;
}

HAL_StatusTypeDef BspBe1732_SetNoBallValueThreshold(uint8_t threshold)
{
    BspBe1732ParamRecord record;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0UL;
    uint32_t address;
    const uint32_t *word;
    HAL_StatusTypeDef status;

    record.magic = BSP_BE1732_PARAM_MAGIC;
    record.version = BSP_BE1732_PARAM_VERSION;
    record.no_ball_value_threshold = threshold;
    record.crc = Be1732ParamCrc(&record);

    if ((be1732_param_loaded_valid != 0U) &&
        (be1732_no_ball_value_threshold == threshold))
    {
        return HAL_OK;
    }

    be1732_last_flash_error = HAL_FLASH_ERROR_NONE;
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        be1732_last_flash_error = HAL_FLASH_GetError();
        return status;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = BSP_BE1732_PARAM_FLASH_SECTOR;
    erase_init.NbSectors = 1UL;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if (status == HAL_OK)
    {
        address = BSP_BE1732_PARAM_FLASH_ADDR;
        word = (const uint32_t *)&record;
        for (uint32_t i = 0UL; i < (sizeof(record) / sizeof(uint32_t)); i++)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word[i]);
            if (status != HAL_OK)
            {
                break;
            }
            address += sizeof(uint32_t);
        }
    }

    if (status != HAL_OK)
    {
        be1732_last_flash_error = HAL_FLASH_GetError();
    }
    (void)HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        return status;
    }

    Be1732LoadParam();
    if (be1732_no_ball_value_threshold != threshold)
    {
        be1732_last_flash_error = HAL_FLASH_ERROR_OPERATION;
        return HAL_ERROR;
    }

    be1732_no_ball_count = 0U;
    be1732_last_valid_channel = 0U;
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
        be1732_no_ball_count = 0U;
        be1732_last_valid_channel = 0U;
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

uint32_t BspBe1732_GetLastFlashError(void)
{
    return be1732_last_flash_error;
}
