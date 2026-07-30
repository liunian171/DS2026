#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* Volatile symbols are intentionally public for OpenOCD inspection. */
extern volatile int32_t g_encoder_left_count;
extern volatile int32_t g_encoder_right_count;
extern volatile uint32_t g_encoder_left_invalid_transition_count;
extern volatile uint32_t g_encoder_right_invalid_transition_count;
extern volatile uint32_t g_encoder_left_invalid_transition_count;
extern volatile uint32_t g_encoder_right_invalid_transition_count;
extern volatile uint32_t g_encoder_left_direction_resync_count;
extern volatile uint32_t g_encoder_right_direction_resync_count;
extern volatile uint32_t g_encoder_left_max_events_per_isr;
extern volatile uint32_t g_encoder_right_max_events_per_isr;

bool BSP_Encoder_Init(void);
void BSP_Encoder_Update(void);
void BSP_Encoder_Reset(void);
void BSP_Encoder_SetExpectedDirection(bool left_wheel, int8_t direction);
int32_t BSP_Encoder_GetLeftCount(void);
int32_t BSP_Encoder_GetRightCount(void);

#endif /* BSP_ENCODER_H */
