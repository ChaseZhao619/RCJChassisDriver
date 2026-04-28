#include "app_pi_comm.h"

#include "app_chassis_task.h"
#include "bsp_suction_motor.h"
#include "bsp_usart.h"
#include "usart.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_PI_COMM_UART_ID BSP_USART_6
#define APP_PI_COMM_RX_RING_SIZE 256U
#define APP_PI_COMM_LINE_SIZE 96U

static uint8_t app_pi_rx_byte;
static volatile uint8_t app_pi_rx_ring[APP_PI_COMM_RX_RING_SIZE];
static volatile uint16_t app_pi_rx_head;
static volatile uint16_t app_pi_rx_tail;
static char app_pi_line[APP_PI_COMM_LINE_SIZE];
static uint16_t app_pi_line_len;
static char app_pi_done_dis_args[APP_PI_COMM_LINE_SIZE];
static char app_pi_done_turn_args[APP_PI_COMM_LINE_SIZE];
static uint8_t app_pi_done_dis_valid;
static uint8_t app_pi_done_turn_valid;

static void StartReceive(void)
{
    (void)HAL_UART_Receive_IT(&huart6, &app_pi_rx_byte, 1U);
}

uint16_t AppPiComm_Crc16Ccitt(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    if (data == NULL)
    {
        return crc;
    }

    for (i = 0U; i < size; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static char *SkipSpaces(char *text)
{
    while ((*text != '\0') && (isspace((unsigned char)*text) != 0))
    {
        text++;
    }

    return text;
}

static char *TrimRight(char *text)
{
    size_t length = strlen(text);

    while ((length > 0U) && (isspace((unsigned char)text[length - 1U]) != 0))
    {
        length--;
        text[length] = '\0';
    }

    return text;
}

static uint8_t ReadFloat(char **cursor, float *value)
{
    char *endptr;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL))
    {
        return 0U;
    }

    *cursor = SkipSpaces(*cursor);
    *value = strtof(*cursor, &endptr);
    if (endptr == *cursor)
    {
        return 0U;
    }

    *cursor = endptr;
    return 1U;
}

static uint8_t ReadUint8(char **cursor, uint8_t *value)
{
    char *endptr;
    unsigned long parsed_value;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL))
    {
        return 0U;
    }

    *cursor = SkipSpaces(*cursor);
    if (**cursor == '-')
    {
        return 0U;
    }

    parsed_value = strtoul(*cursor, &endptr, 10);
    if ((endptr == *cursor) || (parsed_value > 255UL))
    {
        return 0U;
    }

    *cursor = endptr;
    *value = (uint8_t)parsed_value;
    return 1U;
}

static void SendPayloadWithCrc(const char *payload)
{
    char tx[APP_PI_COMM_LINE_SIZE];
    uint16_t crc;
    int length;

    if (payload == NULL)
    {
        return;
    }

    crc = AppPiComm_Crc16Ccitt((const uint8_t *)payload, (uint16_t)strlen(payload));
    length = snprintf(tx, sizeof(tx), "%s *%04X\r\n", payload, crc);
    if (length <= 0)
    {
        return;
    }
    if ((uint32_t)length > sizeof(tx))
    {
        length = (int)sizeof(tx);
    }

    (void)BspUsart_Transmit(APP_PI_COMM_UART_ID, (const uint8_t *)tx, (uint16_t)length, 10U);
}

static void SendCommandError(char *line)
{
    char *command;
    char *end;
    char payload[APP_PI_COMM_LINE_SIZE];

    if (line == NULL)
    {
        SendPayloadWithCrc("cmd eror");
        return;
    }

    TrimRight(line);
    command = SkipSpaces(line);
    end = command;
    while ((*end != '\0') &&
           (isspace((unsigned char)*end) == 0) &&
           (*end != '*'))
    {
        end++;
    }

    if (end == command)
    {
        SendPayloadWithCrc("cmd eror");
        return;
    }

    *end = '\0';
    (void)snprintf(payload, sizeof(payload), "%s eror", command);
    SendPayloadWithCrc(payload);
}

static uint8_t ValidateAndSplitCrc(char *line, char **payload)
{
    char *star;
    char *crc_text;
    char *endptr;
    uint32_t received_crc;
    uint16_t calc_crc;

    if ((line == NULL) || (payload == NULL))
    {
        return 0U;
    }

    TrimRight(line);
    star = strrchr(line, '*');
    if (star == NULL)
    {
        return 0U;
    }

    *star = '\0';
    TrimRight(line);
    crc_text = SkipSpaces(star + 1);
    if (strlen(crc_text) != 4U)
    {
        return 0U;
    }

    received_crc = strtoul(crc_text, &endptr, 16);
    if ((*endptr != '\0') || (received_crc > 0xFFFFUL))
    {
        return 0U;
    }

    calc_crc = AppPiComm_Crc16Ccitt((const uint8_t *)line, (uint16_t)strlen(line));
    if (calc_crc != (uint16_t)received_crc)
    {
        return 0U;
    }

    *payload = SkipSpaces(line);
    return 1U;
}

static uint8_t EnsureLineEnded(char *cursor)
{
    cursor = SkipSpaces(cursor);
    return (*cursor == '\0') ? 1U : 0U;
}

static int32_t FloatToCent(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }

    return (int32_t)((value * 100.0f) - 0.5f);
}

static void FormatCentValue(char *buffer, uint16_t size, int32_t value_cent)
{
    int32_t abs_cent = value_cent;
    const char *sign = "";

    if ((buffer == NULL) || (size == 0U))
    {
        return;
    }

    if (value_cent < 0)
    {
        sign = "-";
        abs_cent = -value_cent;
    }

    (void)snprintf(buffer,
                   size,
                   "%s%ld.%02ld",
                   sign,
                   (long)(abs_cent / 100),
                   (long)(abs_cent % 100));
}

static void CopyCommandArgs(char *dest, uint16_t size, const char *args)
{
    if ((dest == NULL) || (size == 0U))
    {
        return;
    }

    if (args == NULL)
    {
        dest[0] = '\0';
        return;
    }

    (void)snprintf(dest, size, "%s", args);
}

static void SendCommandStateReply(const char *command, const char *state, const char *args)
{
    char response[APP_PI_COMM_LINE_SIZE];

    if ((command == NULL) || (state == NULL))
    {
        return;
    }

    (void)snprintf(response,
                   sizeof(response),
                   "%s %s%s",
                   command,
                   state,
                   (args != NULL) ? args : "");
    SendPayloadWithCrc(response);
}

static void SendSuckCommandReply(const char *state, uint8_t speed_percent)
{
    char response[APP_PI_COMM_LINE_SIZE];

    if (state == NULL)
    {
        return;
    }

    (void)snprintf(response,
                   sizeof(response),
                   "cmd_suck %s %u",
                   state,
                   speed_percent);
    SendPayloadWithCrc(response);
}

static void HandlePayload(char *payload)
{
    char *cursor;
    float x_cm;
    float y_cm;
    float yaw_deg;
    float dx_cm;
    float dy_cm;
    float dyaw_deg;
    float request_yaw_deg;
    uint8_t suck_speed_percent;
    char dx_text[16];
    char dy_text[16];
    char dyaw_text[16];
    char yaw_text[16];
    char response[APP_PI_COMM_LINE_SIZE];
    HAL_StatusTypeDef status;

    if (strncmp(payload, "cmd_dis", 7U) == 0)
    {
        cursor = payload + 7U;
        if ((ReadFloat(&cursor, &x_cm) == 0U) ||
            (ReadFloat(&cursor, &y_cm) == 0U) ||
            (EnsureLineEnded(cursor) == 0U))
        {
            SendPayloadWithCrc("err arg");
            return;
        }

        status = AppChassisTask_CommandDistanceCm(x_cm, y_cm);
        if (status == HAL_OK)
        {
            CopyCommandArgs(app_pi_done_dis_args,
                            sizeof(app_pi_done_dis_args),
                            payload + 7U);
            app_pi_done_dis_valid = 1U;
            app_pi_done_turn_valid = 0U;
            SendCommandStateReply("cmd_dis", "ok", payload + 7U);
        }
        else
        {
            SendCommandStateReply("cmd_dis", "busy", payload + 7U);
        }
        return;
    }

    if (strncmp(payload, "cmd_turn", 8U) == 0)
    {
        cursor = payload + 8U;
        if ((ReadFloat(&cursor, &yaw_deg) == 0U) ||
            (EnsureLineEnded(cursor) == 0U))
        {
            SendPayloadWithCrc("err arg");
            return;
        }

        status = AppChassisTask_CommandTurnDeg(yaw_deg);
        if (status == HAL_OK)
        {
            CopyCommandArgs(app_pi_done_turn_args,
                            sizeof(app_pi_done_turn_args),
                            payload + 8U);
            app_pi_done_turn_valid = 1U;
            app_pi_done_dis_valid = 0U;
            SendCommandStateReply("cmd_turn", "ok", payload + 8U);
        }
        else
        {
            SendCommandStateReply("cmd_turn", "busy", payload + 8U);
        }
        return;
    }

    if (strncmp(payload, "cmd_suck", 8U) == 0)
    {
        cursor = payload + 8U;
        if ((ReadUint8(&cursor, &suck_speed_percent) == 0U) ||
            (suck_speed_percent > 100U) ||
            (EnsureLineEnded(cursor) == 0U))
        {
            SendPayloadWithCrc("err arg");
            return;
        }

        status = BspSuctionMotor_SetSpeedPercent(suck_speed_percent);
        if (status == HAL_OK)
        {
            SendSuckCommandReply("ok", suck_speed_percent);
        }
        else
        {
            SendSuckCommandReply("busy", suck_speed_percent);
        }
        return;
    }

    if (strncmp(payload, "cmd_anglecal", 12U) == 0)
    {
        cursor = payload + 12U;
        if (EnsureLineEnded(cursor) == 0U)
        {
            SendPayloadWithCrc("err arg");
            return;
        }

        status = Main_ResetYawZero();
        if (status == HAL_OK)
        {
            SendCommandStateReply("cmd_anglecal", "ok", "");
            SendCommandStateReply("cmd_anglecal", "done", "");
        }
        else if (status == HAL_BUSY)
        {
            SendCommandStateReply("cmd_anglecal", "busy", "");
        }
        else
        {
            SendPayloadWithCrc("cmd_anglecal eror");
        }
        return;
    }

    if (strncmp(payload, "cmd_mcureset", 12U) == 0)
    {
        cursor = payload + 12U;
        if (EnsureLineEnded(cursor) == 0U)
        {
            SendPayloadWithCrc("err arg");
            return;
        }

        SendCommandStateReply("cmd_mcureset", "ok", "");
        SendCommandStateReply("cmd_mcureset", "done", "");
        HAL_Delay(20U);
        NVIC_SystemReset();
        return;
    }

    if (strncmp(payload, "cmd_request", 11U) == 0)
    {
        cursor = payload + 11U;
        if (EnsureLineEnded(cursor) == 0U)
        {
            SendPayloadWithCrc("err arg");
            return;
        }

        status = AppChassisTask_GetRequestDelta(&dx_cm, &dy_cm, &dyaw_deg, &request_yaw_deg);
        if (status != HAL_OK)
        {
            SendPayloadWithCrc("cmd_request busy");
            return;
        }

        FormatCentValue(dx_text, sizeof(dx_text), FloatToCent(dx_cm));
        FormatCentValue(dy_text, sizeof(dy_text), FloatToCent(dy_cm));
        FormatCentValue(dyaw_text, sizeof(dyaw_text), FloatToCent(dyaw_deg));
        FormatCentValue(yaw_text, sizeof(yaw_text), FloatToCent(request_yaw_deg));
        (void)snprintf(response,
                       sizeof(response),
                       "cmd_request %s %s %s %s",
                       dx_text,
                       dy_text,
                       dyaw_text,
                       yaw_text);
        SendPayloadWithCrc(response);
        return;
    }

    SendPayloadWithCrc("err cmd");
}

static void ProcessLine(char *line)
{
    char *payload = NULL;

    if (ValidateAndSplitCrc(line, &payload) == 0U)
    {
        SendCommandError(line);
        return;
    }

    HandlePayload(payload);
}

static uint8_t PopByte(uint8_t *byte)
{
    if ((byte == NULL) || (app_pi_rx_head == app_pi_rx_tail))
    {
        return 0U;
    }

    *byte = app_pi_rx_ring[app_pi_rx_tail];
    app_pi_rx_tail = (uint16_t)((app_pi_rx_tail + 1U) % APP_PI_COMM_RX_RING_SIZE);
    return 1U;
}

static void SendDoneEvent(void)
{
    switch (AppChassisTask_ConsumeDoneEvent())
    {
    case APP_CHASSIS_TASK_DONE_DIS:
        if (app_pi_done_dis_valid != 0U)
        {
            SendCommandStateReply("cmd_dis", "done", app_pi_done_dis_args);
            app_pi_done_dis_valid = 0U;
        }
        else
        {
            SendPayloadWithCrc("cmd_dis done");
        }
        break;

    case APP_CHASSIS_TASK_DONE_TURN:
        if (app_pi_done_turn_valid != 0U)
        {
            SendCommandStateReply("cmd_turn", "done", app_pi_done_turn_args);
            app_pi_done_turn_valid = 0U;
        }
        else
        {
            SendPayloadWithCrc("cmd_turn done");
        }
        break;

    case APP_CHASSIS_TASK_DONE_NONE:
    default:
        break;
    }
}

void AppPiComm_Init(void)
{
    app_pi_rx_head = 0U;
    app_pi_rx_tail = 0U;
    app_pi_line_len = 0U;
    app_pi_done_dis_args[0] = '\0';
    app_pi_done_turn_args[0] = '\0';
    app_pi_done_dis_valid = 0U;
    app_pi_done_turn_valid = 0U;
    StartReceive();
}

void AppPiComm_Task(void)
{
    uint8_t byte;

    SendDoneEvent();

    while (PopByte(&byte) != 0U)
    {
        if (byte == '\r')
        {
            continue;
        }

        if (byte == '\n')
        {
            app_pi_line[app_pi_line_len] = '\0';
            if (app_pi_line_len > 0U)
            {
                ProcessLine(app_pi_line);
                SendDoneEvent();
            }
            app_pi_line_len = 0U;
            continue;
        }

        if (app_pi_line_len < (APP_PI_COMM_LINE_SIZE - 1U))
        {
            app_pi_line[app_pi_line_len] = (char)byte;
            app_pi_line_len++;
        }
        else
        {
            app_pi_line_len = 0U;
            SendPayloadWithCrc("err long");
            SendDoneEvent();
        }
    }

    SendDoneEvent();
}

void AppPiComm_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if ((huart == NULL) || (huart->Instance != USART6))
    {
        return;
    }

    next_head = (uint16_t)((app_pi_rx_head + 1U) % APP_PI_COMM_RX_RING_SIZE);
    if (next_head != app_pi_rx_tail)
    {
        app_pi_rx_ring[app_pi_rx_head] = app_pi_rx_byte;
        app_pi_rx_head = next_head;
    }

    StartReceive();
}

void AppPiComm_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART6))
    {
        return;
    }

    StartReceive();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    AppPiComm_OnUartRxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    AppPiComm_OnUartError(huart);
}
