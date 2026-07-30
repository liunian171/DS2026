#include "bsp_encoder.h"

#include "bsp_encoder_config.h"
#include "tim.h"

volatile int32_t g_encoder_left_count;
volatile int32_t g_encoder_right_count;
volatile uint32_t g_encoder_left_a_transition_count;
volatile uint32_t g_encoder_left_b_transition_count;
volatile uint32_t g_encoder_right_a_transition_count;
volatile uint32_t g_encoder_right_b_transition_count;
volatile uint8_t g_encoder_left_a_state;
volatile uint8_t g_encoder_left_b_state;
volatile uint8_t g_encoder_right_a_state;
volatile uint8_t g_encoder_right_b_state;

static uint16_t left_previous_raw;
static uint16_t right_previous_raw;
static uint8_t left_a_previous;
static uint8_t left_b_previous;
static uint8_t right_a_previous;
static uint8_t right_b_previous;

bool BSP_Encoder_Init(void)
{
    BSP_Encoder_Reset();

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
        return false;
    }

    return true;
}

void BSP_Encoder_Update(void)
{
    uint16_t left_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t left_delta = (int16_t)(left_raw - left_previous_raw);
    int16_t right_delta = (int16_t)(right_raw - right_previous_raw);
    uint8_t left_a = ((GPIOA->IDR & GPIO_PIN_15) != 0U) ? 1U : 0U;
    uint8_t left_b = ((GPIOB->IDR & GPIO_PIN_3) != 0U) ? 1U : 0U;
    uint8_t right_a = ((GPIOB->IDR & GPIO_PIN_6) != 0U) ? 1U : 0U;
    uint8_t right_b = ((GPIOB->IDR & GPIO_PIN_7) != 0U) ? 1U : 0U;

    if (left_a != left_a_previous)
    {
        g_encoder_left_a_transition_count++;
    }
    if (left_b != left_b_previous)
    {
        g_encoder_left_b_transition_count++;
    }
    if (right_a != right_a_previous)
    {
        g_encoder_right_a_transition_count++;
    }
    if (right_b != right_b_previous)
    {
        g_encoder_right_b_transition_count++;
    }
    left_a_previous = left_a;
    left_b_previous = left_b;
    right_a_previous = right_a;
    right_b_previous = right_b;
    g_encoder_left_a_state = left_a;
    g_encoder_left_b_state = left_b;
    g_encoder_right_a_state = right_a;
    g_encoder_right_b_state = right_b;

    left_previous_raw = left_raw;
    right_previous_raw = right_raw;
    g_encoder_left_count += BSP_ENCODER_LEFT_DIRECTION_SIGN * (int32_t)left_delta;
    g_encoder_right_count += BSP_ENCODER_RIGHT_DIRECTION_SIGN * (int32_t)right_delta;
}

void BSP_Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    left_previous_raw = 0U;
    right_previous_raw = 0U;
    g_encoder_left_count = 0;
    g_encoder_right_count = 0;
    g_encoder_left_a_transition_count = 0U;
    g_encoder_left_b_transition_count = 0U;
    g_encoder_right_a_transition_count = 0U;
    g_encoder_right_b_transition_count = 0U;
    left_a_previous = ((GPIOA->IDR & GPIO_PIN_15) != 0U) ? 1U : 0U;
    left_b_previous = ((GPIOB->IDR & GPIO_PIN_3) != 0U) ? 1U : 0U;
    right_a_previous = ((GPIOB->IDR & GPIO_PIN_6) != 0U) ? 1U : 0U;
    right_b_previous = ((GPIOB->IDR & GPIO_PIN_7) != 0U) ? 1U : 0U;
    g_encoder_left_a_state = left_a_previous;
    g_encoder_left_b_state = left_b_previous;
    g_encoder_right_a_state = right_a_previous;
    g_encoder_right_b_state = right_b_previous;
}

int32_t BSP_Encoder_GetLeftCount(void)
{
    return g_encoder_left_count;
}

int32_t BSP_Encoder_GetRightCount(void)
{
    return g_encoder_right_count;
}
