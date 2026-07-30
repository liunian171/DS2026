#include "line_sensor.h"

#include "bsp_line_uart.h"

#include <string.h>

#define LINE_FRAME_LENGTH 11U
#define LINE_HEADER       0xFFU
#define LINE_TAIL         0xFEU

typedef struct
{
    uint8_t frame[LINE_FRAME_LENGTH];
    uint8_t index;
} LineParser;

static LineParser line_parser;

volatile uint32_t g_line_valid_frame_count;
volatile uint32_t g_line_checksum_error_count;
volatile uint32_t g_line_format_error_count;
volatile uint32_t g_line_last_update_ms;
volatile uint32_t g_line_channel_mask;
volatile uint32_t g_line_detected_mask;
volatile uint32_t g_line_active_count;
volatile uint32_t g_line_lost;
volatile int32_t g_line_position_x1000;
volatile uint32_t g_line_detection_event_count;
volatile uint32_t g_line_last_detected_mask;
volatile int32_t g_line_last_detected_position_x1000;
volatile uint32_t g_line_last_detection_ms;
volatile uint32_t g_line_d1;
volatile uint32_t g_line_d2;
volatile uint32_t g_line_d3;
volatile uint32_t g_line_d4;
volatile uint32_t g_line_d5;
volatile uint32_t g_line_d6;
volatile uint32_t g_line_d7;
volatile uint32_t g_line_d8;

static void LineSensor_ResetParser(uint8_t possible_header)
{
    line_parser.index = 0U;
    if (possible_header == LINE_HEADER)
    {
        line_parser.frame[0] = LINE_HEADER;
        line_parser.index = 1U;
    }
}

static void LineSensor_Publish(uint32_t now_ms)
{
    static const int32_t used_weights_x1000[6] = {
        -2500, -1500, -500, 500, 1500, 2500
    };
    volatile uint32_t *const channels[8] = {
        &g_line_d1, &g_line_d2, &g_line_d3, &g_line_d4,
        &g_line_d5, &g_line_d6, &g_line_d7, &g_line_d8
    };
    uint32_t mask = 0U;
    uint32_t detected_mask = 0U;
    uint32_t detected_count = 0U;
    int32_t weighted_sum = 0;
    uint8_t index;

    for (index = 0U; index < 8U; ++index)
    {
        uint32_t value = line_parser.frame[index + 1U];
        *channels[index] = value;
        if (value != 0U)
        {
            mask |= (1UL << index);
        }
    }

    /* Only D2..D7 participate in control. Black line is active low. */
    for (index = 1U; index <= 6U; ++index)
    {
        if (line_parser.frame[index + 1U] == 0U)
        {
            detected_mask |= (1UL << index);
            detected_count++;
            weighted_sum += used_weights_x1000[index - 1U];
        }
    }

    g_line_channel_mask = mask;
    g_line_detected_mask = detected_mask;
    g_line_active_count = detected_count;
    g_line_lost = (detected_count == 0U) ? 1U : 0U;
    if (detected_count > 0U)
    {
        g_line_position_x1000 = weighted_sum / (int32_t)detected_count;
        g_line_last_detected_mask = detected_mask;
        g_line_last_detected_position_x1000 = g_line_position_x1000;
        g_line_last_detection_ms = now_ms;
        g_line_detection_event_count++;
    }
    g_line_last_update_ms = now_ms;
    g_line_valid_frame_count++;
}

static void LineSensor_InputByte(uint8_t byte, uint32_t now_ms)
{
    uint8_t checksum;
    uint8_t index;

    if (line_parser.index == 0U)
    {
        LineSensor_ResetParser(byte);
        return;
    }

    if (line_parser.index <= 8U)
    {
        if (byte > 1U)
        {
            g_line_format_error_count++;
            LineSensor_ResetParser(byte);
            return;
        }
        line_parser.frame[line_parser.index++] = byte;
        return;
    }

    if (line_parser.index == 9U)
    {
        checksum = 0U;
        for (index = 1U; index <= 8U; ++index)
        {
            checksum = (uint8_t)(checksum + line_parser.frame[index]);
        }
        if (byte != checksum)
        {
            g_line_checksum_error_count++;
            LineSensor_ResetParser(byte);
            return;
        }
        line_parser.frame[line_parser.index++] = byte;
        return;
    }

    if (byte != LINE_TAIL)
    {
        g_line_format_error_count++;
        LineSensor_ResetParser(byte);
        return;
    }

    line_parser.frame[10] = byte;
    LineSensor_Publish(now_ms);
    LineSensor_ResetParser(0U);
}

bool LineSensor_Init(void)
{
    memset(&line_parser, 0, sizeof(line_parser));
    g_line_valid_frame_count = 0U;
    g_line_checksum_error_count = 0U;
    g_line_format_error_count = 0U;
    g_line_last_update_ms = 0U;
    g_line_channel_mask = 0U;
    g_line_detected_mask = 0U;
    g_line_active_count = 0U;
    g_line_lost = 1U;
    g_line_position_x1000 = 0;
    g_line_detection_event_count = 0U;
    g_line_last_detected_mask = 0U;
    g_line_last_detected_position_x1000 = 0;
    g_line_last_detection_ms = 0U;
    g_line_d1 = 0U;
    g_line_d2 = 0U;
    g_line_d3 = 0U;
    g_line_d4 = 0U;
    g_line_d5 = 0U;
    g_line_d6 = 0U;
    g_line_d7 = 0U;
    g_line_d8 = 0U;
    return true;
}

void LineSensor_Process(uint32_t now_ms)
{
    uint8_t byte;
    while (BSP_LineUART_ReadByte(&byte))
    {
        LineSensor_InputByte(byte, now_ms);
    }
}

bool LineSensor_IsFresh(uint32_t now_ms, uint32_t maximum_age_ms)
{
    return (g_line_valid_frame_count > 0U) &&
           ((uint32_t)(now_ms - g_line_last_update_ms) <= maximum_age_ms);
}
