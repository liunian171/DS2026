#ifndef CYZ_SENSOR_H
#define CYZ_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CYZ_INTERFACE_UART = 0,
    CYZ_INTERFACE_I2C
} CYZ_Interface;

typedef enum
{
    CYZ_REPORT_RATE_50_HZ = 0x00,
    CYZ_REPORT_RATE_20_HZ = 0x01,
    CYZ_REPORT_RATE_10_HZ = 0x02,
    CYZ_REPORT_RATE_QUERY = 0x03
} CYZ_ReportRate;

extern volatile uint32_t g_cyz_valid_frame_count;
extern volatile uint32_t g_cyz_telemetry_frame_count;
extern volatile uint32_t g_cyz_crc_error_count;
extern volatile uint32_t g_cyz_format_error_count;
extern volatile uint32_t g_cyz_sequence_drop_count;
extern volatile uint32_t g_cyz_last_update_ms;
extern volatile uint16_t g_cyz_sequence;
extern volatile int32_t g_cyz_angle_mdeg;
extern volatile int32_t g_cyz_gyro_mdps;
extern volatile uint32_t g_cyz_ack_count;
extern volatile uint8_t g_cyz_last_ack_command;
extern volatile uint8_t g_cyz_last_ack_result;
extern volatile uint16_t g_cyz_i2c_status;
extern volatile uint16_t g_cyz_i2c_sample_hz;
extern volatile int16_t g_cyz_i2c_temperature_centi_c;
extern volatile uint32_t g_cyz_i2c_request_count;
extern volatile uint32_t g_cyz_i2c_error_count;
extern volatile uint8_t g_cyz_i2c_busy;

bool CYZ_Sensor_Init(CYZ_Interface interface);
void CYZ_Sensor_Process(uint32_t now_ms);
bool CYZ_Sensor_IsFresh(uint32_t now_ms, uint32_t maximum_age_ms);
bool CYZ_Sensor_ZeroAngle(void);
bool CYZ_Sensor_ReestimateBias(void);
bool CYZ_Sensor_SetReportRate(CYZ_ReportRate rate);
bool CYZ_Sensor_QueryTelemetry(void);
void CYZ_Sensor_I2CMemoryReadComplete(void);
void CYZ_Sensor_I2CError(void);

#endif /* CYZ_SENSOR_H */
