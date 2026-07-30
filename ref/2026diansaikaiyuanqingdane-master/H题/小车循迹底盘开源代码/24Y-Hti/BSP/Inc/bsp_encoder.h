#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_encoder_left_a_transition_count;
extern volatile uint32_t g_encoder_left_b_transition_count;
extern volatile uint32_t g_encoder_right_a_transition_count;
extern volatile uint32_t g_encoder_right_b_transition_count;
extern volatile uint8_t g_encoder_left_a_state;
extern volatile uint8_t g_encoder_left_b_state;
extern volatile uint8_t g_encoder_right_a_state;
extern volatile uint8_t g_encoder_right_b_state;

/* Volatile symbols are intentionally public for OpenOCD inspection. */
extern volatile int32_t g_encoder_left_count;
extern volatile int32_t g_encoder_right_count;

bool BSP_Encoder_Init(void);
void BSP_Encoder_Update(void);
void BSP_Encoder_Reset(void);
int32_t BSP_Encoder_GetLeftCount(void);
int32_t BSP_Encoder_GetRightCount(void);

#endif /* BSP_ENCODER_H */
