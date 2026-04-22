#ifndef BSP_BNO085_H
#define BSP_BNO085_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BNO085_I2C_ADDR_LOW  0x4AU
#define BNO085_I2C_ADDR_HIGH 0x4BU

#ifndef BNO085_INT_GPIO_Port
#ifdef BNO_INT2_GPIO_Port
#define BNO085_INT_GPIO_Port BNO_INT2_GPIO_Port
#else
#ifdef BNO_INT_GPIO_Port
#define BNO085_INT_GPIO_Port BNO_INT_GPIO_Port
#else
#define BNO085_INT_GPIO_Port GPIOB
#endif
#endif
#endif

#ifndef BNO085_INT_Pin
#ifdef BNO_INT2_Pin
#define BNO085_INT_Pin BNO_INT2_Pin
#else
#ifdef BNO_INT_Pin
#define BNO085_INT_Pin BNO_INT_Pin
#else
#define BNO085_INT_Pin GPIO_PIN_1
#endif
#endif
#endif

#ifndef BNO085_NRST_GPIO_Port
#ifdef BNO_NRST_GPIO_Port
#define BNO085_NRST_GPIO_Port BNO_NRST_GPIO_Port
#else
#define BNO085_NRST_GPIO_Port GPIOB
#endif
#endif

#ifndef BNO085_NRST_Pin
#ifdef BNO_NRST_Pin
#define BNO085_NRST_Pin BNO_NRST_Pin
#else
#define BNO085_NRST_Pin GPIO_PIN_8
#endif
#endif

#ifndef BNO085_KEY_GPIO_Port
#ifdef BNO_KEY_GPIO_Port
#define BNO085_KEY_GPIO_Port BNO_KEY_GPIO_Port
#else
#define BNO085_KEY_GPIO_Port GPIOE
#endif
#endif

#ifndef BNO085_KEY_Pin
#ifdef BNO_KEY_Pin
#define BNO085_KEY_Pin BNO_KEY_Pin
#else
#define BNO085_KEY_Pin GPIO_PIN_13
#endif
#endif

#define BNO085_ROTATION_VECTOR_REPORT_ID      0x05U
#define BNO085_ACCELEROMETER_REPORT_ID        0x01U
#define BNO085_GYROSCOPE_REPORT_ID            0x02U
#define BNO085_MAGNETOMETER_REPORT_ID         0x03U
#define BNO085_LINEAR_ACCEL_REPORT_ID         0x04U
#define BNO085_GRAVITY_REPORT_ID              0x06U
#define BNO085_GAME_ROTATION_VECTOR_REPORT_ID 0x08U

typedef struct
{
    uint8_t i2c_addr;
    uint8_t reset_cause;
    uint8_t sw_major;
    uint8_t sw_minor;
    uint16_t sw_patch;
    uint32_t sw_part_number;
    uint32_t sw_build_number;
} Bno085ProductId;

typedef struct
{
    uint8_t report_id;
    uint8_t sequence;
    uint8_t status;
    int16_t i_raw;
    int16_t j_raw;
    int16_t k_raw;
    int16_t real_raw;
    int16_t accuracy_raw;
    float i;
    float j;
    float k;
    float real;
    float accuracy;
} Bno085RotationVector;

typedef struct
{
    uint8_t report_id;
    uint8_t sequence;
    uint8_t status;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    float x;
    float y;
    float z;
} Bno085Vector3;

typedef struct
{
    Bno085RotationVector rotation;
    Bno085Vector3 accel;
    Bno085Vector3 gyro;
    Bno085Vector3 mag;
    Bno085Vector3 linear_accel;
    Bno085Vector3 gravity;
    uint8_t has_rotation;
    uint8_t has_accel;
    uint8_t has_gyro;
    uint8_t has_mag;
    uint8_t has_linear_accel;
    uint8_t has_gravity;
} Bno085SensorData;

typedef enum
{
    BNO085_ERROR_NONE = 0,
    BNO085_ERROR_I2C_REINIT,
    BNO085_ERROR_PRODUCT_ID_TX,
    BNO085_ERROR_WAIT_PRODUCT_ID,
    BNO085_ERROR_PRODUCT_ID_LENGTH,
    BNO085_ERROR_PACKET_NOT_READY,
    BNO085_ERROR_PACKET_HEADER_RX,
    BNO085_ERROR_PACKET_HEADER_LENGTH,
    BNO085_ERROR_PACKET_BODY_RX,
    BNO085_ERROR_PACKET_BODY_LENGTH
} Bno085Error;

typedef struct
{
    HAL_StatusTypeDef ready_4b;
    HAL_StatusTypeDef ready_4a;
    uint32_t error_4b;
    uint32_t error_4a;
} Bno085I2cProbeResult;

HAL_StatusTypeDef Bno085_Init(void);
void Bno085_HardwareReset(void);
void Bno085_ClearI2cBus(void);
HAL_StatusTypeDef Bno085_GetProductId(Bno085ProductId *product_id);
HAL_StatusTypeDef Bno085_EnableReport(uint8_t report_id, uint32_t interval_us);
HAL_StatusTypeDef Bno085_EnableDefaultReports(uint32_t interval_us);
HAL_StatusTypeDef Bno085_EnableRotationVector(uint32_t interval_us);
HAL_StatusTypeDef Bno085_EnableGameRotationVector(uint32_t interval_us);
HAL_StatusTypeDef Bno085_ReadRotationVector(Bno085RotationVector *rotation_vector);
HAL_StatusTypeDef Bno085_ReadSensorData(Bno085SensorData *sensor_data);
HAL_StatusTypeDef Bno085_GetYawDegrees(const Bno085RotationVector *rotation_vector, float *yaw_deg);
HAL_StatusTypeDef Bno085_GetYawDegX100(const Bno085RotationVector *rotation_vector, int32_t *yaw_deg_x100);
HAL_StatusTypeDef Bno085_SetYawZero(const Bno085RotationVector *rotation_vector);
uint8_t Bno085_IsZeroKeyPressed(void);
uint8_t Bno085_GetI2cAddress(void);
Bno085Error Bno085_GetLastError(void);
uint32_t Bno085_GetLastHalI2cError(void);
uint8_t Bno085_GetIntPinLevel(void);
void Bno085_GetI2cProbeResult(Bno085I2cProbeResult *result);
uint16_t Bno085_GetLastPacketLength(void);
void Bno085_GetLastHeader(uint8_t header[4]);
uint8_t Bno085_GetLastChannel(void);
uint8_t Bno085_GetLastReportId(void);
void Bno085_GetLastPayload(uint8_t *payload, uint16_t max_length);

#ifdef __cplusplus
}
#endif

#endif
