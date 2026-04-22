#include "bsp_bno085.h"

#include "i2c.h"
#include <math.h>
#include <string.h>

#define BNO085_I2C_TIMEOUT_MS       30U
#define BNO085_COMMAND_TIMEOUT_MS   500U
#define BNO085_I2C_READY_TIMEOUT_MS 100U
#define BNO085_PACKET_PAYLOAD_SIZE  512U
#define BNO085_SHTP_HEADER_SIZE     4U
#define BNO085_FULL_PACKET_SIZE     (BNO085_PACKET_PAYLOAD_SIZE + BNO085_SHTP_HEADER_SIZE)
#define BNO085_Q_POINT_14_SCALE     (1.0f / 16384.0f)
#define BNO085_ACCEL_SCALE          (1.0f / 256.0f)
#define BNO085_GYRO_SCALE           (1.0f / 512.0f)
#define BNO085_MAG_SCALE            (1.0f / 16.0f)
#define BNO085_RAD_TO_DEG           (57.2957795f)

#define SHTP_CHANNEL_COMMAND        0U
#define SHTP_CHANNEL_CONTROL        2U
#define SHTP_CHANNEL_REPORTS        3U
#define SHTP_CHANNEL_WAKE_REPORTS   4U
#define SHTP_CHANNEL_GYRO           5U

#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8U
#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9U
#define SHTP_REPORT_BASE_TIMESTAMP      0xFBU
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFDU

typedef struct
{
    uint8_t channel;
    uint16_t length;
    uint8_t payload[BNO085_PACKET_PAYLOAD_SIZE];
} Bno085Packet;

static uint8_t bno085_i2c_addr = BNO085_I2C_ADDR_HIGH;
static uint8_t bno085_sequence[6];
static float bno085_yaw_zero_deg;
static Bno085Error bno085_last_error = BNO085_ERROR_NONE;
static uint32_t bno085_last_hal_i2c_error;
static Bno085I2cProbeResult bno085_probe_result;
static uint8_t bno085_last_header[BNO085_SHTP_HEADER_SIZE];
static uint16_t bno085_last_packet_length;
static uint8_t bno085_last_channel;
static uint8_t bno085_last_report_id;
static uint8_t bno085_last_payload[64];
static uint16_t bno085_last_payload_length;

static uint16_t bno085_le_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t bno085_le_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int16_t bno085_le_i16(const uint8_t *data)
{
    return (int16_t)bno085_le_u16(data);
}

static uint16_t bno085_hal_addr(void)
{
    return (uint16_t)bno085_i2c_addr << 1;
}

static HAL_StatusTypeDef bno085_probe_address(uint8_t address, uint32_t *hal_error)
{
    HAL_StatusTypeDef status;

    bno085_i2c_addr = address;
    status = HAL_I2C_IsDeviceReady(&hi2c1, bno085_hal_addr(), 3U, BNO085_I2C_READY_TIMEOUT_MS);
    if (hal_error != NULL)
    {
        *hal_error = HAL_I2C_GetError(&hi2c1);
    }

    return status;
}

static HAL_StatusTypeDef bno085_send_packet(uint8_t channel, const uint8_t *payload, uint16_t length)
{
    uint8_t packet[BNO085_SHTP_HEADER_SIZE + 32U];
    uint16_t packet_length = length + BNO085_SHTP_HEADER_SIZE;

    if ((payload == NULL) || (length == 0U) || (length > 32U) || (channel >= sizeof(bno085_sequence)))
    {
        return HAL_ERROR;
    }

    packet[0] = (uint8_t)(packet_length & 0xFFU);
    packet[1] = (uint8_t)(packet_length >> 8);
    packet[2] = channel;
    packet[3] = bno085_sequence[channel]++;
    memcpy(&packet[BNO085_SHTP_HEADER_SIZE], payload, length);

    if (HAL_I2C_Master_Transmit(&hi2c1, bno085_hal_addr(), packet, packet_length, BNO085_I2C_TIMEOUT_MS) != HAL_OK)
    {
        bno085_last_hal_i2c_error = HAL_I2C_GetError(&hi2c1);
        return HAL_ERROR;
    }

    return HAL_OK;
}

static float bno085_wrap_360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void bno085_init_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = BNO085_NRST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BNO085_NRST_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(BNO085_NRST_GPIO_Port, BNO085_NRST_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = BNO085_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BNO085_INT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BNO085_KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BNO085_KEY_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t bno085_data_ready(void)
{
    return HAL_GPIO_ReadPin(BNO085_INT_GPIO_Port, BNO085_INT_Pin) == GPIO_PIN_RESET;
}

static HAL_StatusTypeDef bno085_read_packet(Bno085Packet *packet)
{
    uint8_t header[BNO085_SHTP_HEADER_SIZE];
    uint8_t full_packet[BNO085_FULL_PACKET_SIZE];
    uint16_t total_length;

    if (packet == NULL)
    {
        return HAL_ERROR;
    }

    if (bno085_data_ready() == 0U)
    {
        bno085_last_error = BNO085_ERROR_PACKET_NOT_READY;
        return HAL_BUSY;
    }

    if (HAL_I2C_Master_Receive(&hi2c1, bno085_hal_addr(), header, sizeof(header), BNO085_I2C_TIMEOUT_MS) != HAL_OK)
    {
        bno085_last_error = BNO085_ERROR_PACKET_HEADER_RX;
        bno085_last_hal_i2c_error = HAL_I2C_GetError(&hi2c1);
        return HAL_ERROR;
    }

    memcpy(bno085_last_header, header, sizeof(bno085_last_header));
    total_length = bno085_le_u16(header) & 0x7FFFU;
    bno085_last_packet_length = total_length;
    if ((total_length < BNO085_SHTP_HEADER_SIZE) || (total_length > BNO085_FULL_PACKET_SIZE))
    {
        bno085_last_error = BNO085_ERROR_PACKET_HEADER_LENGTH;
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(&hi2c1, bno085_hal_addr(), full_packet, total_length, BNO085_I2C_TIMEOUT_MS) != HAL_OK)
    {
        bno085_last_error = BNO085_ERROR_PACKET_BODY_RX;
        bno085_last_hal_i2c_error = HAL_I2C_GetError(&hi2c1);
        return HAL_ERROR;
    }

    total_length = bno085_le_u16(full_packet) & 0x7FFFU;
    bno085_last_packet_length = total_length;
    if ((total_length < BNO085_SHTP_HEADER_SIZE) || (total_length > BNO085_FULL_PACKET_SIZE))
    {
        bno085_last_error = BNO085_ERROR_PACKET_BODY_LENGTH;
        return HAL_ERROR;
    }

    packet->length = total_length - BNO085_SHTP_HEADER_SIZE;
    packet->channel = full_packet[2];
    bno085_last_channel = packet->channel;

    if (packet->length > 0U)
    {
        memcpy(packet->payload, &full_packet[BNO085_SHTP_HEADER_SIZE], packet->length);
    }

    bno085_last_report_id = (packet->length > 0U) ? packet->payload[0] : 0U;
    bno085_last_payload_length = (packet->length < sizeof(bno085_last_payload)) ?
                                 packet->length : (uint16_t)sizeof(bno085_last_payload);
    if (bno085_last_payload_length > 0U)
    {
        memcpy(bno085_last_payload, packet->payload, bno085_last_payload_length);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef bno085_wait_for_packet(uint8_t report_id, uint8_t channel, Bno085Packet *packet, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if (bno085_read_packet(packet) == HAL_OK)
        {
            if ((packet->channel == channel) && (packet->length > 0U) && (packet->payload[0] == report_id))
            {
                return HAL_OK;
            }
        }

        HAL_Delay(2U);
    }

    return HAL_TIMEOUT;
}

static uint8_t bno085_sensor_report_length(uint8_t report_id)
{
    switch (report_id)
    {
    case BNO085_GAME_ROTATION_VECTOR_REPORT_ID:
        return 12U;
    case BNO085_ROTATION_VECTOR_REPORT_ID:
        return 14U;
    case 0x01U:
    case 0x02U:
    case 0x03U:
    case 0x04U:
    case 0x06U:
        return 14U;
    case 0x09U:
        return 12U;
    case 0x0EU:
    case 0x0FU:
    case 0x10U:
        return 6U;
    case 0x11U:
        return 5U;
    case 0x13U:
    case 0x14U:
    case 0x15U:
        return 1U;
    default:
        return 0U;
    }
}

static void bno085_drain_startup_packets(uint32_t timeout_ms)
{
    Bno085Packet packet;
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if (bno085_data_ready() == 0U)
        {
            HAL_Delay(2U);
            continue;
        }

        if (bno085_read_packet(&packet) != HAL_OK)
        {
            HAL_Delay(2U);
        }
    }

    bno085_last_error = BNO085_ERROR_NONE;
    bno085_last_hal_i2c_error = HAL_I2C_ERROR_NONE;
}

static uint8_t bno085_parse_rotation_payload(const uint8_t *payload, uint16_t length, Bno085RotationVector *rotation_vector)
{
    uint16_t offset = 0U;

    if ((payload == NULL) || (rotation_vector == NULL))
    {
        return 0U;
    }

    if ((length > 5U) && (payload[0] == SHTP_REPORT_BASE_TIMESTAMP))
    {
        offset = 5U;
    }

    if ((length >= (offset + 12U)) && (payload[offset] == BNO085_GAME_ROTATION_VECTOR_REPORT_ID))
    {
        rotation_vector->report_id = payload[offset];
        rotation_vector->sequence = payload[offset + 1U];
        rotation_vector->status = payload[offset + 2U] & 0x03U;
        rotation_vector->i_raw = bno085_le_i16(&payload[offset + 4U]);
        rotation_vector->j_raw = bno085_le_i16(&payload[offset + 6U]);
        rotation_vector->k_raw = bno085_le_i16(&payload[offset + 8U]);
        rotation_vector->real_raw = bno085_le_i16(&payload[offset + 10U]);
        rotation_vector->accuracy_raw = 0;
    }
    else if ((length >= (offset + 14U)) && (payload[offset] == BNO085_ROTATION_VECTOR_REPORT_ID))
    {
        rotation_vector->report_id = payload[offset];
        rotation_vector->sequence = payload[offset + 1U];
        rotation_vector->status = payload[offset + 2U] & 0x03U;
        rotation_vector->i_raw = bno085_le_i16(&payload[offset + 4U]);
        rotation_vector->j_raw = bno085_le_i16(&payload[offset + 6U]);
        rotation_vector->k_raw = bno085_le_i16(&payload[offset + 8U]);
        rotation_vector->real_raw = bno085_le_i16(&payload[offset + 10U]);
        rotation_vector->accuracy_raw = bno085_le_i16(&payload[offset + 12U]);
    }
    else
    {
        return 0U;
    }

    rotation_vector->i = (float)rotation_vector->i_raw * BNO085_Q_POINT_14_SCALE;
    rotation_vector->j = (float)rotation_vector->j_raw * BNO085_Q_POINT_14_SCALE;
    rotation_vector->k = (float)rotation_vector->k_raw * BNO085_Q_POINT_14_SCALE;
    rotation_vector->real = (float)rotation_vector->real_raw * BNO085_Q_POINT_14_SCALE;
    rotation_vector->accuracy = (float)rotation_vector->accuracy_raw * BNO085_Q_POINT_14_SCALE;

    return 1U;
}

static uint8_t bno085_parse_vector3_report(const uint8_t *payload, uint16_t offset, uint16_t length, float scale, Bno085Vector3 *vector)
{
    if ((payload == NULL) || (vector == NULL) || (length < (offset + 10U)))
    {
        return 0U;
    }

    vector->report_id = payload[offset];
    vector->sequence = payload[offset + 1U];
    vector->status = payload[offset + 2U] & 0x03U;
    vector->x_raw = bno085_le_i16(&payload[offset + 4U]);
    vector->y_raw = bno085_le_i16(&payload[offset + 6U]);
    vector->z_raw = bno085_le_i16(&payload[offset + 8U]);
    vector->x = (float)vector->x_raw * scale;
    vector->y = (float)vector->y_raw * scale;
    vector->z = (float)vector->z_raw * scale;

    return 1U;
}

static uint8_t bno085_parse_sensor_payload(const uint8_t *payload, uint16_t length, Bno085SensorData *sensor_data)
{
    uint16_t offset = 0U;
    uint8_t parsed = 0U;

    if ((payload == NULL) || (sensor_data == NULL))
    {
        return 0U;
    }

    if ((length > 5U) && (payload[0] == SHTP_REPORT_BASE_TIMESTAMP))
    {
        offset = 5U;
    }

    while (offset < length)
    {
        uint8_t report_id = payload[offset];
        uint8_t report_length = bno085_sensor_report_length(report_id);

        if ((report_length == 0U) || (length < (offset + report_length)))
        {
            break;
        }

        switch (report_id)
        {
        case BNO085_GAME_ROTATION_VECTOR_REPORT_ID:
        case BNO085_ROTATION_VECTOR_REPORT_ID:
            if (bno085_parse_rotation_payload(&payload[offset], report_length, &sensor_data->rotation) != 0U)
            {
                sensor_data->has_rotation = 1U;
                parsed = 1U;
            }
            break;
        case BNO085_ACCELEROMETER_REPORT_ID:
            if (bno085_parse_vector3_report(payload, offset, length, BNO085_ACCEL_SCALE, &sensor_data->accel) != 0U)
            {
                sensor_data->has_accel = 1U;
                parsed = 1U;
            }
            break;
        case BNO085_GYROSCOPE_REPORT_ID:
            if (bno085_parse_vector3_report(payload, offset, length, BNO085_GYRO_SCALE, &sensor_data->gyro) != 0U)
            {
                sensor_data->has_gyro = 1U;
                parsed = 1U;
            }
            break;
        case BNO085_MAGNETOMETER_REPORT_ID:
            if (bno085_parse_vector3_report(payload, offset, length, BNO085_MAG_SCALE, &sensor_data->mag) != 0U)
            {
                sensor_data->has_mag = 1U;
                parsed = 1U;
            }
            break;
        case BNO085_LINEAR_ACCEL_REPORT_ID:
            if (bno085_parse_vector3_report(payload, offset, length, BNO085_ACCEL_SCALE, &sensor_data->linear_accel) != 0U)
            {
                sensor_data->has_linear_accel = 1U;
                parsed = 1U;
            }
            break;
        case BNO085_GRAVITY_REPORT_ID:
            if (bno085_parse_vector3_report(payload, offset, length, BNO085_ACCEL_SCALE, &sensor_data->gravity) != 0U)
            {
                sensor_data->has_gravity = 1U;
                parsed = 1U;
            }
            break;
        default:
            break;
        }

        offset += report_length;
    }

    return parsed;
}

uint8_t Bno085_GetI2cAddress(void)
{
    return bno085_i2c_addr;
}

Bno085Error Bno085_GetLastError(void)
{
    return bno085_last_error;
}

uint32_t Bno085_GetLastHalI2cError(void)
{
    return bno085_last_hal_i2c_error;
}

uint8_t Bno085_GetIntPinLevel(void)
{
    return HAL_GPIO_ReadPin(BNO085_INT_GPIO_Port, BNO085_INT_Pin) == GPIO_PIN_SET;
}

void Bno085_GetI2cProbeResult(Bno085I2cProbeResult *result)
{
    if (result != NULL)
    {
        *result = bno085_probe_result;
    }
}

uint16_t Bno085_GetLastPacketLength(void)
{
    return bno085_last_packet_length;
}

void Bno085_GetLastHeader(uint8_t header[4])
{
    if (header != NULL)
    {
        memcpy(header, bno085_last_header, sizeof(bno085_last_header));
    }
}

uint8_t Bno085_GetLastChannel(void)
{
    return bno085_last_channel;
}

uint8_t Bno085_GetLastReportId(void)
{
    return bno085_last_report_id;
}

void Bno085_GetLastPayload(uint8_t *payload, uint16_t max_length)
{
    uint16_t length = bno085_last_payload_length;

    if (payload == NULL)
    {
        return;
    }

    if (length > max_length)
    {
        length = max_length;
    }

    if (length > 0U)
    {
        memcpy(payload, bno085_last_payload, length);
    }
}

HAL_StatusTypeDef Bno085_Init(void)
{
    Bno085ProductId product_id;

    bno085_last_error = BNO085_ERROR_NONE;
    bno085_last_hal_i2c_error = HAL_I2C_ERROR_NONE;
    memset(&bno085_probe_result, 0, sizeof(bno085_probe_result));

    bno085_init_pins();
    Bno085_ClearI2cBus();
    Bno085_HardwareReset();
    HAL_I2C_DeInit(&hi2c1);
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        bno085_last_error = BNO085_ERROR_I2C_REINIT;
        bno085_last_hal_i2c_error = HAL_I2C_GetError(&hi2c1);
        return HAL_ERROR;
    }

    memset(bno085_sequence, 0, sizeof(bno085_sequence));

    HAL_Delay(300U);

    bno085_probe_result.ready_4b = bno085_probe_address(BNO085_I2C_ADDR_HIGH, &bno085_probe_result.error_4b);
    bno085_probe_result.ready_4a = bno085_probe_address(BNO085_I2C_ADDR_LOW, &bno085_probe_result.error_4a);

    if (bno085_probe_result.ready_4b == HAL_OK)
    {
        bno085_i2c_addr = BNO085_I2C_ADDR_HIGH;
        memset(bno085_sequence, 0, sizeof(bno085_sequence));
        bno085_drain_startup_packets(150U);
        if (Bno085_GetProductId(&product_id) == HAL_OK)
        {
            return HAL_OK;
        }
    }

    if (bno085_probe_result.ready_4a == HAL_OK)
    {
        bno085_i2c_addr = BNO085_I2C_ADDR_LOW;
        memset(bno085_sequence, 0, sizeof(bno085_sequence));
        bno085_drain_startup_packets(150U);
        if (Bno085_GetProductId(&product_id) == HAL_OK)
        {
            return HAL_OK;
        }
    }

    if ((bno085_probe_result.ready_4b != HAL_OK) && (bno085_probe_result.ready_4a != HAL_OK))
    {
        bno085_last_error = BNO085_ERROR_PRODUCT_ID_TX;
        bno085_last_hal_i2c_error = (bno085_probe_result.error_4b != HAL_I2C_ERROR_NONE) ?
                                    bno085_probe_result.error_4b : bno085_probe_result.error_4a;
    }

    return HAL_ERROR;
}

void Bno085_HardwareReset(void)
{
    HAL_GPIO_WritePin(BNO085_NRST_GPIO_Port, BNO085_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(BNO085_NRST_GPIO_Port, BNO085_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(300U);
}

void Bno085_ClearI2cBus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t i;

    HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1U);

    for (i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1U);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1U);
}

HAL_StatusTypeDef Bno085_GetProductId(Bno085ProductId *product_id)
{
    const uint8_t request[] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0x00U};
    Bno085Packet packet;
    HAL_StatusTypeDef status;

    if (product_id == NULL)
    {
        return HAL_ERROR;
    }

    if (bno085_send_packet(SHTP_CHANNEL_CONTROL, request, sizeof(request)) != HAL_OK)
    {
        bno085_last_error = BNO085_ERROR_PRODUCT_ID_TX;
        return HAL_ERROR;
    }

    status = bno085_wait_for_packet(SHTP_REPORT_PRODUCT_ID_RESPONSE, SHTP_CHANNEL_CONTROL, &packet, BNO085_COMMAND_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        if ((bno085_last_error == BNO085_ERROR_NONE) || (bno085_last_error == BNO085_ERROR_PACKET_NOT_READY))
        {
            bno085_last_error = BNO085_ERROR_WAIT_PRODUCT_ID;
        }
        return status;
    }

    if (packet.length < 16U)
    {
        bno085_last_error = BNO085_ERROR_PRODUCT_ID_LENGTH;
        return HAL_ERROR;
    }

    product_id->i2c_addr = bno085_i2c_addr;
    product_id->reset_cause = packet.payload[1];
    product_id->sw_major = packet.payload[2];
    product_id->sw_minor = packet.payload[3];
    product_id->sw_part_number = bno085_le_u32(&packet.payload[4]);
    product_id->sw_build_number = bno085_le_u32(&packet.payload[8]);
    product_id->sw_patch = bno085_le_u16(&packet.payload[12]);

    return HAL_OK;
}

HAL_StatusTypeDef Bno085_EnableReport(uint8_t report_id, uint32_t interval_us)
{
    uint8_t command[17] = {0};

    command[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    command[1] = report_id;
    command[5] = (uint8_t)(interval_us & 0xFFU);
    command[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    command[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    command[8] = (uint8_t)((interval_us >> 24) & 0xFFU);

    return bno085_send_packet(SHTP_CHANNEL_CONTROL, command, sizeof(command));
}

HAL_StatusTypeDef Bno085_EnableDefaultReports(uint32_t interval_us)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (Bno085_EnableReport(BNO085_GAME_ROTATION_VECTOR_REPORT_ID, interval_us) != HAL_OK)
    {
        status = HAL_ERROR;
    }
    HAL_Delay(2U);
    if (Bno085_EnableReport(BNO085_GYROSCOPE_REPORT_ID, interval_us) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef Bno085_EnableRotationVector(uint32_t interval_us)
{
    return Bno085_EnableReport(BNO085_ROTATION_VECTOR_REPORT_ID, interval_us);
}

HAL_StatusTypeDef Bno085_EnableGameRotationVector(uint32_t interval_us)
{
    return Bno085_EnableReport(BNO085_GAME_ROTATION_VECTOR_REPORT_ID, interval_us);
}

HAL_StatusTypeDef Bno085_ReadRotationVector(Bno085RotationVector *rotation_vector)
{
    Bno085Packet packet;

    if (rotation_vector == NULL)
    {
        return HAL_ERROR;
    }

    if (bno085_read_packet(&packet) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (((packet.channel == SHTP_CHANNEL_REPORTS) ||
         (packet.channel == SHTP_CHANNEL_WAKE_REPORTS) ||
         (packet.channel == SHTP_CHANNEL_GYRO)) &&
        (bno085_parse_rotation_payload(packet.payload, packet.length, rotation_vector) != 0U))
    {
        return HAL_OK;
    }

    return HAL_BUSY;
}

HAL_StatusTypeDef Bno085_ReadSensorData(Bno085SensorData *sensor_data)
{
    Bno085Packet packet;

    if (sensor_data == NULL)
    {
        return HAL_ERROR;
    }

    if (bno085_read_packet(&packet) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (((packet.channel == SHTP_CHANNEL_REPORTS) ||
         (packet.channel == SHTP_CHANNEL_WAKE_REPORTS) ||
         (packet.channel == SHTP_CHANNEL_GYRO)) &&
        (bno085_parse_sensor_payload(packet.payload, packet.length, sensor_data) != 0U))
    {
        return HAL_OK;
    }

    return HAL_BUSY;
}

HAL_StatusTypeDef Bno085_GetYawDegrees(const Bno085RotationVector *rotation_vector, float *yaw_deg)
{
    float raw_yaw;
    float siny_cosp;
    float cosy_cosp;

    if ((rotation_vector == NULL) || (yaw_deg == NULL))
    {
        return HAL_ERROR;
    }

    siny_cosp = 2.0f * ((rotation_vector->real * rotation_vector->k) +
                        (rotation_vector->i * rotation_vector->j));
    cosy_cosp = 1.0f - (2.0f * ((rotation_vector->j * rotation_vector->j) +
                                (rotation_vector->k * rotation_vector->k)));
    raw_yaw = atan2f(siny_cosp, cosy_cosp) * BNO085_RAD_TO_DEG;

    *yaw_deg = bno085_wrap_360(raw_yaw - bno085_yaw_zero_deg);

    return HAL_OK;
}

HAL_StatusTypeDef Bno085_GetYawDegX100(const Bno085RotationVector *rotation_vector, int32_t *yaw_deg_x100)
{
    float yaw_deg;

    if (yaw_deg_x100 == NULL)
    {
        return HAL_ERROR;
    }

    if (Bno085_GetYawDegrees(rotation_vector, &yaw_deg) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *yaw_deg_x100 = (int32_t)(yaw_deg * 100.0f);

    return HAL_OK;
}

HAL_StatusTypeDef Bno085_SetYawZero(const Bno085RotationVector *rotation_vector)
{
    float current_yaw;
    float old_zero = bno085_yaw_zero_deg;

    bno085_yaw_zero_deg = 0.0f;
    if (Bno085_GetYawDegrees(rotation_vector, &current_yaw) != HAL_OK)
    {
        bno085_yaw_zero_deg = old_zero;
        return HAL_ERROR;
    }

    bno085_yaw_zero_deg = current_yaw;

    return HAL_OK;
}

uint8_t Bno085_IsZeroKeyPressed(void)
{
    return HAL_GPIO_ReadPin(BNO085_KEY_GPIO_Port, BNO085_KEY_Pin) == GPIO_PIN_RESET;
}
