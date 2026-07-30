#include "cyz_sensor.h"

#include "bsp_gyro_uart.h"
#include "bsp_i2c.h"
#include "i2c.h"

#include <limits.h>
#include <string.h>

#define CYZ_TELEMETRY_FRAME_SIZE 16U
#define CYZ_ACK_FRAME_SIZE        8U
#define CYZ_SCALE_FRAME_SIZE      12U
#define CYZ_COMMAND_FRAME_SIZE    8U

#define CYZ_CMD_ZERO_OR_BIAS      0x01U
#define CYZ_CMD_SET_RATE          0x03U
#define CYZ_CMD_QUERY_TELEMETRY   0x04U

#define CYZ_I2C_ADDRESS_7BIT             0x42U
#define CYZ_I2C_DATA_REGISTER            0x00U
#define CYZ_I2C_COMMAND_REGISTER         0x20U
#define CYZ_I2C_DATA_SIZE                20U
#define CYZ_I2C_CRC_DATA_SIZE            18U
#define CYZ_I2C_DEFAULT_READ_INTERVAL_MS 20U
#define CYZ_I2C_TRANSFER_TIMEOUT_MS      10U

#define CYZ_I2C_WHO_AM_I_VALUE           0x71U
#define CYZ_I2C_PROTOCOL_VERSION         0x01U
#define CYZ_I2C_STATUS_DATA_VALID        (1U << 0)
#define CYZ_I2C_STATUS_SENSOR_ERROR      (1U << 3)

typedef struct
{
    uint8_t buffer[CYZ_TELEMETRY_FRAME_SIZE];
    uint8_t position;
    uint8_t expected_length;
    uint8_t command_sequence;
    bool has_sequence;
} CYZ_Parser;

typedef struct
{
    uint16_t status;
    uint16_t sequence;
    uint16_t sample_hz;
    int32_t angle_mdeg;
    int32_t gyro_mdps;
    int16_t temperature_centi_c;
} CYZ_I2CData;

typedef enum
{
    CYZ_I2C_PARSE_OK = 0,
    CYZ_I2C_PARSE_FORMAT_ERROR,
    CYZ_I2C_PARSE_CRC_ERROR
} CYZ_I2CParseResult;

static CYZ_Parser cyz_parser;
static CYZ_Interface cyz_interface;
static uint8_t cyz_i2c_buffer[CYZ_I2C_DATA_SIZE];
static uint32_t cyz_i2c_last_request_ms;
static uint32_t cyz_i2c_read_interval_ms;

volatile uint32_t g_cyz_valid_frame_count;
volatile uint32_t g_cyz_telemetry_frame_count;
volatile uint32_t g_cyz_crc_error_count;
volatile uint32_t g_cyz_format_error_count;
volatile uint32_t g_cyz_sequence_drop_count;
volatile uint32_t g_cyz_last_update_ms;
volatile uint16_t g_cyz_sequence;
volatile int32_t g_cyz_angle_mdeg;
volatile int32_t g_cyz_gyro_mdps;
volatile uint32_t g_cyz_ack_count;
volatile uint8_t g_cyz_last_ack_command;
volatile uint8_t g_cyz_last_ack_result;
volatile uint16_t g_cyz_i2c_status;
volatile uint16_t g_cyz_i2c_sample_hz;
volatile int16_t g_cyz_i2c_temperature_centi_c;
volatile uint32_t g_cyz_i2c_request_count;
volatile uint32_t g_cyz_i2c_error_count;
volatile uint8_t g_cyz_i2c_busy;

static uint16_t CYZ_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t CYZ_ReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int16_t CYZ_ReadS16LE(const uint8_t *data)
{
    uint16_t raw = CYZ_ReadU16LE(data);
    int16_t value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static int32_t CYZ_ReadS32LE(const uint8_t *data)
{
    uint32_t raw = CYZ_ReadU32LE(data);
    int32_t value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint16_t CYZ_CRC16Modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static bool CYZ_CentiToMilli(int32_t centi_value, int32_t *milli_value)
{
    int64_t scaled = (int64_t)centi_value * 10;

    if ((scaled > INT32_MAX) || (scaled < INT32_MIN))
    {
        return false;
    }
    *milli_value = (int32_t)scaled;
    return true;
}

static CYZ_I2CParseResult CYZ_ParseI2CData(const uint8_t *data,
                                            CYZ_I2CData *sample)
{
    int32_t angle_cdeg;
    int32_t gyro_cdps;

    if ((data[0] != CYZ_I2C_WHO_AM_I_VALUE) ||
        (data[1] != CYZ_I2C_PROTOCOL_VERSION))
    {
        return CYZ_I2C_PARSE_FORMAT_ERROR;
    }
    if (CYZ_ReadU16LE(&data[18]) !=
        CYZ_CRC16Modbus(data, CYZ_I2C_CRC_DATA_SIZE))
    {
        return CYZ_I2C_PARSE_CRC_ERROR;
    }

    sample->status = CYZ_ReadU16LE(&data[2]);
    if (((sample->status & CYZ_I2C_STATUS_DATA_VALID) == 0U) ||
        ((sample->status & CYZ_I2C_STATUS_SENSOR_ERROR) != 0U))
    {
        return CYZ_I2C_PARSE_FORMAT_ERROR;
    }

    sample->sequence = CYZ_ReadU16LE(&data[4]);
    sample->sample_hz = CYZ_ReadU16LE(&data[6]);
    angle_cdeg = CYZ_ReadS32LE(&data[8]);
    gyro_cdps = CYZ_ReadS32LE(&data[12]);
    sample->temperature_centi_c = CYZ_ReadS16LE(&data[16]);
    if (!CYZ_CentiToMilli(angle_cdeg, &sample->angle_mdeg) ||
        !CYZ_CentiToMilli(gyro_cdps, &sample->gyro_mdps))
    {
        return CYZ_I2C_PARSE_FORMAT_ERROR;
    }
    return CYZ_I2C_PARSE_OK;
}

static void CYZ_PublishI2CData(const CYZ_I2CData *sample, uint32_t now_ms)
{
    /* The sensor sequence runs at 500 Hz while the host polls at 50 Hz.
       Deliberately skipped internal samples are not communication drops. */
    cyz_parser.has_sequence = true;
    g_cyz_sequence = sample->sequence;
    g_cyz_angle_mdeg = sample->angle_mdeg;
    g_cyz_gyro_mdps = sample->gyro_mdps;
    g_cyz_i2c_status = sample->status;
    g_cyz_i2c_sample_hz = sample->sample_hz;
    g_cyz_i2c_temperature_centi_c = sample->temperature_centi_c;
    g_cyz_last_update_ms = now_ms;
    g_cyz_telemetry_frame_count++;
    g_cyz_valid_frame_count++;
}

static bool CYZ_StartI2CRead(uint32_t now_ms)
{
    HAL_StatusTypeDef status;

    if (g_cyz_i2c_busy != 0U)
    {
        return false;
    }

    g_cyz_i2c_busy = 1U;
    cyz_i2c_last_request_ms = now_ms;
    g_cyz_i2c_request_count++;
    status = HAL_I2C_Mem_Read_IT(&hi2c1,
                                 (uint16_t)(CYZ_I2C_ADDRESS_7BIT << 1),
                                 CYZ_I2C_DATA_REGISTER,
                                 I2C_MEMADD_SIZE_8BIT,
                                 cyz_i2c_buffer,
                                 CYZ_I2C_DATA_SIZE);
    if (status != HAL_OK)
    {
        g_cyz_i2c_busy = 0U;
        g_cyz_i2c_error_count++;
        return false;
    }
    return true;
}

static bool CYZ_FloatToMilli(uint32_t raw, int32_t *milli_value)
{
    float value;

    if ((raw & 0x7F800000UL) == 0x7F800000UL)
    {
        return false;
    }

    memcpy(&value, &raw, sizeof(value));
    if ((value > 2147483.0f) || (value < -2147483.0f))
    {
        return false;
    }
    *milli_value = (int32_t)((value >= 0.0f) ?
                             (value * 1000.0f + 0.5f) :
                             (value * 1000.0f - 0.5f));
    return true;
}

static bool CYZ_ParseTelemetry(const uint8_t *frame,
                               uint16_t *sequence,
                               int32_t *angle_mdeg,
                               int32_t *gyro_mdps)
{
    if ((frame[0] != 0xAAU) || (frame[1] != 0x55U) ||
        (frame[14] != 0x55U) || (frame[15] != 0xAAU))
    {
        return false;
    }
    if (CYZ_ReadU16LE(&frame[12]) != CYZ_CRC16Modbus(&frame[2], 10U))
    {
        return false;
    }

    *sequence = CYZ_ReadU16LE(&frame[2]);
    return CYZ_FloatToMilli(CYZ_ReadU32LE(&frame[4]), angle_mdeg) &&
           CYZ_FloatToMilli(CYZ_ReadU32LE(&frame[8]), gyro_mdps);
}

static void CYZ_ResetStream(uint8_t possible_header)
{
    cyz_parser.position = 0U;
    cyz_parser.expected_length = 0U;
    if ((possible_header == 0xAAU) || (possible_header == 0xA5U))
    {
        cyz_parser.buffer[0] = possible_header;
        cyz_parser.position = 1U;
    }
}

static void CYZ_HandleCompleteFrame(uint32_t now_ms)
{
    uint8_t *frame = cyz_parser.buffer;

    if ((frame[0] == 0xAAU) &&
        (cyz_parser.expected_length == CYZ_TELEMETRY_FRAME_SIZE))
    {
        uint16_t sequence;
        int32_t angle_mdeg;
        int32_t gyro_mdps;

        if (CYZ_ReadU16LE(&frame[12]) != CYZ_CRC16Modbus(&frame[2], 10U))
        {
            g_cyz_crc_error_count++;
        }
        else if (!CYZ_ParseTelemetry(frame, &sequence,
                                     &angle_mdeg, &gyro_mdps))
        {
            g_cyz_format_error_count++;
        }
        else
        {
            if (cyz_parser.has_sequence)
            {
                g_cyz_sequence_drop_count +=
                    (uint16_t)(sequence - g_cyz_sequence - 1U);
            }
            cyz_parser.has_sequence = true;
            g_cyz_sequence = sequence;
            g_cyz_angle_mdeg = angle_mdeg;
            g_cyz_gyro_mdps = gyro_mdps;
            g_cyz_last_update_ms = now_ms;
            g_cyz_telemetry_frame_count++;
            g_cyz_valid_frame_count++;
        }
    }
    else if ((frame[0] == 0xA5U) && (frame[1] == 0x5BU) &&
             (cyz_parser.expected_length == CYZ_ACK_FRAME_SIZE))
    {
        if (CYZ_ReadU16LE(&frame[5]) != CYZ_CRC16Modbus(&frame[2], 3U))
        {
            g_cyz_crc_error_count++;
        }
        else if (frame[7] != 0x5BU)
        {
            g_cyz_format_error_count++;
        }
        else
        {
            g_cyz_last_ack_command = frame[2];
            g_cyz_last_ack_result = frame[3];
            g_cyz_ack_count++;
            g_cyz_valid_frame_count++;
        }
    }
    else if ((frame[0] == 0xA5U) && (frame[1] == 0x5CU) &&
             (cyz_parser.expected_length == CYZ_SCALE_FRAME_SIZE))
    {
        if ((frame[11] == 0x5CU) &&
            (CYZ_ReadU16LE(&frame[9]) == CYZ_CRC16Modbus(&frame[2], 7U)))
        {
            g_cyz_valid_frame_count++;
        }
        else
        {
            g_cyz_crc_error_count++;
        }
    }
    else
    {
        g_cyz_format_error_count++;
    }
}

static void CYZ_InputByte(uint8_t byte, uint32_t now_ms)
{
    if (cyz_parser.position == 0U)
    {
        CYZ_ResetStream(byte);
        return;
    }

    if (cyz_parser.position == 1U)
    {
        if ((cyz_parser.buffer[0] == 0xAAU) && (byte == 0x55U))
        {
            cyz_parser.expected_length = CYZ_TELEMETRY_FRAME_SIZE;
        }
        else if ((cyz_parser.buffer[0] == 0xA5U) && (byte == 0x5BU))
        {
            cyz_parser.expected_length = CYZ_ACK_FRAME_SIZE;
        }
        else if ((cyz_parser.buffer[0] == 0xA5U) && (byte == 0x5CU))
        {
            cyz_parser.expected_length = CYZ_SCALE_FRAME_SIZE;
        }
        else
        {
            g_cyz_format_error_count++;
            CYZ_ResetStream(byte);
            return;
        }
    }

    cyz_parser.buffer[cyz_parser.position++] = byte;
    if (cyz_parser.position >= cyz_parser.expected_length)
    {
        CYZ_HandleCompleteFrame(now_ms);
        CYZ_ResetStream(0U);
    }
}

static bool CYZ_SendCommand(uint8_t command, uint8_t parameter)
{
    uint8_t frame[CYZ_COMMAND_FRAME_SIZE];
    uint16_t crc;

    if (cyz_interface == CYZ_INTERFACE_I2C)
    {
        if ((command != CYZ_CMD_ZERO_OR_BIAS) ||
            (g_cyz_i2c_busy != 0U))
        {
            return false;
        }
        return BSP_I2C_MemoryWrite(CYZ_I2C_ADDRESS_7BIT,
                                   CYZ_I2C_COMMAND_REGISTER,
                                   I2C_MEMADD_SIZE_8BIT,
                                   &parameter, 1U,
                                   CYZ_I2C_TRANSFER_TIMEOUT_MS);
    }

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = command;
    frame[3] = parameter;
    frame[4] = cyz_parser.command_sequence++;
    crc = CYZ_CRC16Modbus(&frame[2], 3U);
    frame[5] = (uint8_t)crc;
    frame[6] = (uint8_t)(crc >> 8);
    frame[7] = 0x5AU;
    return BSP_GyroUART_Write(frame, sizeof(frame));
}

static bool CYZ_ProtocolSelfTest(void)
{
    static const uint8_t sample[CYZ_TELEMETRY_FRAME_SIZE] = {
        0xAAU, 0x55U, 0x01U, 0x00U,
        0x00U, 0x00U, 0x80U, 0x3FU,
        0x00U, 0x00U, 0x00U, 0x00U,
        0x2AU, 0x07U, 0x55U, 0xAAU
    };
    uint16_t sequence;
    int32_t angle_mdeg;
    int32_t gyro_mdps;

    return CYZ_ParseTelemetry(sample, &sequence, &angle_mdeg, &gyro_mdps) &&
           (sequence == 1U) && (angle_mdeg == 1000) && (gyro_mdps == 0);
}

static bool CYZ_I2CProtocolSelfTest(void)
{
    uint8_t sample[CYZ_I2C_DATA_SIZE] = {
        0x71U, 0x01U, 0x01U, 0x00U,
        0x01U, 0x00U, 0xF4U, 0x01U,
        0x64U, 0x00U, 0x00U, 0x00U,
        0xE7U, 0xFFU, 0xFFU, 0xFFU,
        0xC4U, 0x09U, 0x00U, 0x00U
    };
    CYZ_I2CData parsed;
    uint16_t crc = CYZ_CRC16Modbus(sample, CYZ_I2C_CRC_DATA_SIZE);

    sample[18] = (uint8_t)crc;
    sample[19] = (uint8_t)(crc >> 8);
    return (CYZ_ParseI2CData(sample, &parsed) == CYZ_I2C_PARSE_OK) &&
           (parsed.sequence == 1U) &&
           (parsed.sample_hz == 500U) &&
           (parsed.angle_mdeg == 1000) &&
           (parsed.gyro_mdps == -250) &&
           (parsed.temperature_centi_c == 2500);
}

bool CYZ_Sensor_Init(CYZ_Interface interface)
{
    if ((interface != CYZ_INTERFACE_UART) &&
        (interface != CYZ_INTERFACE_I2C))
    {
        return false;
    }

    memset(&cyz_parser, 0, sizeof(cyz_parser));
    memset(cyz_i2c_buffer, 0, sizeof(cyz_i2c_buffer));
    cyz_interface = interface;
    cyz_i2c_read_interval_ms = CYZ_I2C_DEFAULT_READ_INTERVAL_MS;
    cyz_i2c_last_request_ms =
        HAL_GetTick() - CYZ_I2C_DEFAULT_READ_INTERVAL_MS;
    g_cyz_valid_frame_count = 0U;
    g_cyz_telemetry_frame_count = 0U;
    g_cyz_crc_error_count = 0U;
    g_cyz_format_error_count = 0U;
    g_cyz_sequence_drop_count = 0U;
    g_cyz_last_update_ms = 0U;
    g_cyz_sequence = 0U;
    g_cyz_angle_mdeg = 0;
    g_cyz_gyro_mdps = 0;
    g_cyz_ack_count = 0U;
    g_cyz_last_ack_command = 0U;
    g_cyz_last_ack_result = 0U;
    g_cyz_i2c_status = 0U;
    g_cyz_i2c_sample_hz = 0U;
    g_cyz_i2c_temperature_centi_c = 0;
    g_cyz_i2c_request_count = 0U;
    g_cyz_i2c_error_count = 0U;
    g_cyz_i2c_busy = 0U;
    return CYZ_ProtocolSelfTest() && CYZ_I2CProtocolSelfTest();
}

void CYZ_Sensor_Process(uint32_t now_ms)
{
    uint8_t byte;

    if (cyz_interface == CYZ_INTERFACE_I2C)
    {
        if ((g_cyz_i2c_busy == 0U) &&
            ((uint32_t)(now_ms - cyz_i2c_last_request_ms) >=
             cyz_i2c_read_interval_ms))
        {
            (void)CYZ_StartI2CRead(now_ms);
        }
        return;
    }

    while (BSP_GyroUART_ReadByte(&byte))
    {
        CYZ_InputByte(byte, now_ms);
    }
}

bool CYZ_Sensor_IsFresh(uint32_t now_ms, uint32_t maximum_age_ms)
{
    return (g_cyz_telemetry_frame_count > 0U) &&
           ((uint32_t)(now_ms - g_cyz_last_update_ms) <= maximum_age_ms);
}

bool CYZ_Sensor_ZeroAngle(void)
{
    return CYZ_SendCommand(CYZ_CMD_ZERO_OR_BIAS, 0x01U);
}

bool CYZ_Sensor_ReestimateBias(void)
{
    return CYZ_SendCommand(CYZ_CMD_ZERO_OR_BIAS, 0x02U);
}

bool CYZ_Sensor_SetReportRate(CYZ_ReportRate rate)
{
    if ((uint8_t)rate > (uint8_t)CYZ_REPORT_RATE_QUERY)
    {
        return false;
    }
    if (cyz_interface == CYZ_INTERFACE_I2C)
    {
        if (rate == CYZ_REPORT_RATE_50_HZ)
        {
            cyz_i2c_read_interval_ms = 20U;
        }
        else if (rate == CYZ_REPORT_RATE_20_HZ)
        {
            cyz_i2c_read_interval_ms = 50U;
        }
        else if (rate == CYZ_REPORT_RATE_10_HZ)
        {
            cyz_i2c_read_interval_ms = 100U;
        }
        return true;
    }
    return CYZ_SendCommand(CYZ_CMD_SET_RATE, (uint8_t)rate);
}

bool CYZ_Sensor_QueryTelemetry(void)
{
    if (cyz_interface == CYZ_INTERFACE_I2C)
    {
        return CYZ_StartI2CRead(HAL_GetTick());
    }
    return CYZ_SendCommand(CYZ_CMD_QUERY_TELEMETRY, 0x00U);
}

void CYZ_Sensor_I2CMemoryReadComplete(void)
{
    CYZ_I2CData sample;
    CYZ_I2CParseResult result;

    if (cyz_interface != CYZ_INTERFACE_I2C)
    {
        return;
    }

    g_cyz_i2c_busy = 0U;
    result = CYZ_ParseI2CData(cyz_i2c_buffer, &sample);
    if (result == CYZ_I2C_PARSE_OK)
    {
        CYZ_PublishI2CData(&sample, HAL_GetTick());
    }
    else if (result == CYZ_I2C_PARSE_CRC_ERROR)
    {
        g_cyz_crc_error_count++;
    }
    else
    {
        g_cyz_format_error_count++;
    }
}

void CYZ_Sensor_I2CError(void)
{
    if (cyz_interface == CYZ_INTERFACE_I2C)
    {
        g_cyz_i2c_busy = 0U;
        g_cyz_i2c_error_count++;
    }
}
