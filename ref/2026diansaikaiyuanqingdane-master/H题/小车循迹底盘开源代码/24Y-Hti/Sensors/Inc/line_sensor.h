#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_line_valid_frame_count;
extern volatile uint32_t g_line_checksum_error_count;
extern volatile uint32_t g_line_format_error_count;
extern volatile uint32_t g_line_last_update_ms;
extern volatile uint32_t g_line_channel_mask;
extern volatile uint32_t g_line_detected_mask;
extern volatile uint32_t g_line_active_count;
extern volatile uint32_t g_line_lost;
extern volatile int32_t g_line_position_x1000;
extern volatile uint32_t g_line_detection_event_count;
extern volatile uint32_t g_line_last_detected_mask;
extern volatile int32_t g_line_last_detected_position_x1000;
extern volatile uint32_t g_line_last_detection_ms;
extern volatile uint32_t g_line_d1;
extern volatile uint32_t g_line_d2;
extern volatile uint32_t g_line_d3;
extern volatile uint32_t g_line_d4;
extern volatile uint32_t g_line_d5;
extern volatile uint32_t g_line_d6;
extern volatile uint32_t g_line_d7;
extern volatile uint32_t g_line_d8;

bool LineSensor_Init(void);
void LineSensor_Process(uint32_t now_ms);
bool LineSensor_IsFresh(uint32_t now_ms, uint32_t maximum_age_ms);

#endif /* LINE_SENSOR_H */
