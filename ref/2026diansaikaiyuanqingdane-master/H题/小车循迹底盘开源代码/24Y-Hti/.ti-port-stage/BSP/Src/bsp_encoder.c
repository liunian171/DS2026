#include "bsp_encoder.h"

#include "bsp_encoder_config.h"
#include "ti_msp_dl_config.h"

volatile int32_t g_encoder_left_count;
volatile int32_t g_encoder_right_count;
volatile uint32_t g_encoder_left_invalid_transition_count;
volatile uint32_t g_encoder_right_invalid_transition_count;
volatile uint32_t g_encoder_left_direction_resync_count;
volatile uint32_t g_encoder_right_direction_resync_count;
volatile uint32_t g_encoder_left_max_events_per_isr;
volatile uint32_t g_encoder_right_max_events_per_isr;

static volatile uint8_t left_previous_state;
static volatile uint8_t right_previous_state;
static volatile int8_t left_expected_direction;
static volatile int8_t right_expected_direction;

static void Encoder_EnableInputPullups(void)
{
    /* Keep the timer capture mux active while enabling a defined idle level. */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_CAPTURE_ENCODER_M1_C0_IOMUX,
        GPIO_CAPTURE_ENCODER_M1_C0_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_CAPTURE_ENCODER_M1_C1_IOMUX,
        GPIO_CAPTURE_ENCODER_M1_C1_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_CAPTURE_ENCODER_M2_C0_IOMUX,
        GPIO_CAPTURE_ENCODER_M2_C0_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_CAPTURE_ENCODER_M2_C1_IOMUX,
        GPIO_CAPTURE_ENCODER_M2_C1_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* Index is previous_AB << 2 | current_AB. */
static const int8_t quadrature_delta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static uint8_t Encoder_ReadLeftState(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_CAPTURE_ENCODER_M1_C0_PORT,
        GPIO_CAPTURE_ENCODER_M1_C0_PIN | GPIO_CAPTURE_ENCODER_M1_C1_PIN);
    uint8_t a = ((pins & GPIO_CAPTURE_ENCODER_M1_C0_PIN) != 0U) ? 1U : 0U;
    uint8_t b = ((pins & GPIO_CAPTURE_ENCODER_M1_C1_PIN) != 0U) ? 1U : 0U;
    return (uint8_t)((a << 1U) | b);
}

static uint8_t Encoder_ReadRightState(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_CAPTURE_ENCODER_M2_C0_PORT,
        GPIO_CAPTURE_ENCODER_M2_C0_PIN | GPIO_CAPTURE_ENCODER_M2_C1_PIN);
    uint8_t a = ((pins & GPIO_CAPTURE_ENCODER_M2_C0_PIN) != 0U) ? 1U : 0U;
    uint8_t b = ((pins & GPIO_CAPTURE_ENCODER_M2_C1_PIN) != 0U) ? 1U : 0U;
    return (uint8_t)((a << 1U) | b);
}

static void Encoder_ProcessLeftEdge(uint8_t phase_mask)
{
    uint8_t current = (uint8_t)(left_previous_state ^ phase_mask);
    uint8_t transition = (uint8_t)((left_previous_state << 2U) | current);
    int32_t normalized_delta =
        BSP_ENCODER_LEFT_DIRECTION_SIGN * (int32_t)quadrature_delta[transition];

    if (normalized_delta == 0) {
        ++g_encoder_left_invalid_transition_count;
    } else if ((left_expected_direction != 0) &&
               (normalized_delta != left_expected_direction)) {
        /* A delayed or lost CC event can shift the reconstructed phase by
           one bit. The motor command supplies the trusted direction while
           moving; correct this edge instead of feeding a false reverse speed
           to the PI controller. */
        normalized_delta = left_expected_direction;
        ++g_encoder_left_direction_resync_count;
    }

    g_encoder_left_count += normalized_delta;
    left_previous_state = current;
}

static void Encoder_ProcessRightEdge(uint8_t phase_mask)
{
    uint8_t current = (uint8_t)(right_previous_state ^ phase_mask);
    uint8_t transition = (uint8_t)((right_previous_state << 2U) | current);
    int32_t normalized_delta =
        BSP_ENCODER_RIGHT_DIRECTION_SIGN * (int32_t)quadrature_delta[transition];

    if (normalized_delta == 0) {
        ++g_encoder_right_invalid_transition_count;
    } else if ((right_expected_direction != 0) &&
               (normalized_delta != right_expected_direction)) {
        normalized_delta = right_expected_direction;
        ++g_encoder_right_direction_resync_count;
    }

    g_encoder_right_count += normalized_delta;
    right_previous_state = current;
}

bool BSP_Encoder_Init(void)
{
    /* Read the real stationary A/B state before restoring timer capture mux. */
    DL_TimerG_stopCounter(CAPTURE_ENCODER_M1_INST);
    DL_TimerG_stopCounter(CAPTURE_ENCODER_M2_INST);
    DL_GPIO_initDigitalInputFeatures(GPIO_CAPTURE_ENCODER_M1_C0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_CAPTURE_ENCODER_M1_C1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_CAPTURE_ENCODER_M2_C0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_CAPTURE_ENCODER_M2_C1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    left_previous_state = Encoder_ReadLeftState();
    right_previous_state = Encoder_ReadRightState();

    Encoder_EnableInputPullups();
    DL_TimerG_setCCPDirection(CAPTURE_ENCODER_M1_INST,
        DL_TIMER_CC0_INPUT | DL_TIMER_CC1_INPUT);
    DL_TimerG_setCCPDirection(CAPTURE_ENCODER_M2_INST,
        DL_TIMER_CC0_INPUT | DL_TIMER_CC1_INPUT);

    /* Three consecutive timer-clock samples reject narrow motor noise. */
    DL_TimerG_setCaptureCompareInputFilter(CAPTURE_ENCODER_M1_INST,
        DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER,
        DL_TIMER_CC_INPUT_FILT_FP_PER_3, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareInputFilter(CAPTURE_ENCODER_M1_INST,
        DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER,
        DL_TIMER_CC_INPUT_FILT_FP_PER_3, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setCaptureCompareInputFilter(CAPTURE_ENCODER_M2_INST,
        DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER,
        DL_TIMER_CC_INPUT_FILT_FP_PER_3, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareInputFilter(CAPTURE_ENCODER_M2_INST,
        DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER,
        DL_TIMER_CC_INPUT_FILT_FP_PER_3, DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableCaptureCompareInputFilter(
        CAPTURE_ENCODER_M1_INST, DL_TIMER_CC_0_INDEX);
    DL_TimerG_enableCaptureCompareInputFilter(
        CAPTURE_ENCODER_M1_INST, DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableCaptureCompareInputFilter(
        CAPTURE_ENCODER_M2_INST, DL_TIMER_CC_0_INDEX);
    DL_TimerG_enableCaptureCompareInputFilter(
        CAPTURE_ENCODER_M2_INST, DL_TIMER_CC_1_INDEX);

    BSP_Encoder_Reset();
    left_expected_direction = 0;
    right_expected_direction = 0;

    DL_TimerG_disableInterrupt(CAPTURE_ENCODER_M1_INST,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT | DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
    DL_TimerG_disableInterrupt(CAPTURE_ENCODER_M2_INST,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT | DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
    DL_TimerG_clearInterruptStatus(CAPTURE_ENCODER_M1_INST,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT | DL_TIMERG_INTERRUPT_CC1_DN_EVENT |
        DL_TIMERG_INTERRUPT_CC0_UP_EVENT | DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
    DL_TimerG_clearInterruptStatus(CAPTURE_ENCODER_M2_INST,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT | DL_TIMERG_INTERRUPT_CC1_DN_EVENT |
        DL_TIMERG_INTERRUPT_CC0_UP_EVENT | DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
    DL_TimerG_enableInterrupt(CAPTURE_ENCODER_M1_INST,
        DL_TIMERG_INTERRUPT_CC0_UP_EVENT | DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
    DL_TimerG_enableInterrupt(CAPTURE_ENCODER_M2_INST,
        DL_TIMERG_INTERRUPT_CC0_UP_EVENT | DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
    NVIC_ClearPendingIRQ(CAPTURE_ENCODER_M1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(CAPTURE_ENCODER_M2_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_ENCODER_M1_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_ENCODER_M2_INST_INT_IRQN);
    DL_TimerG_startCounter(CAPTURE_ENCODER_M1_INST);
    DL_TimerG_startCounter(CAPTURE_ENCODER_M2_INST);
    return true;
}

void BSP_Encoder_Update(void)
{
    /* Counts are updated immediately by the two timer capture ISRs. */
}

void BSP_Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_encoder_left_count = 0;
    g_encoder_right_count = 0;
    g_encoder_left_invalid_transition_count = 0U;
    g_encoder_right_invalid_transition_count = 0U;
    g_encoder_left_direction_resync_count = 0U;
    g_encoder_right_direction_resync_count = 0U;
    g_encoder_left_max_events_per_isr = 0U;
    g_encoder_right_max_events_per_isr = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

void BSP_Encoder_SetExpectedDirection(bool left_wheel, int8_t direction)
{
    int8_t normalized = (direction > 0) ? 1 : ((direction < 0) ? -1 : 0);

    if (left_wheel) {
        left_expected_direction = normalized;
    } else {
        right_expected_direction = normalized;
    }
}

int32_t BSP_Encoder_GetLeftCount(void)
{
    return g_encoder_left_count;
}

int32_t BSP_Encoder_GetRightCount(void)
{
    return g_encoder_right_count;
}

void CAPTURE_ENCODER_M1_INST_IRQHandler(void)
{
    uint32_t event_count = 0U;
    DL_TIMER_IIDX pending;

    while ((uint32_t)(pending =
               DL_TimerG_getPendingInterrupt(CAPTURE_ENCODER_M1_INST)) != 0U) {
        if (pending == DL_TIMERG_IIDX_CC0_UP) {
            Encoder_ProcessLeftEdge(2U);
            event_count++;
        } else if (pending == DL_TIMERG_IIDX_CC1_UP) {
            Encoder_ProcessLeftEdge(1U);
            event_count++;
        }
    }
    if (event_count > g_encoder_left_max_events_per_isr) {
        g_encoder_left_max_events_per_isr = event_count;
    }
}

void CAPTURE_ENCODER_M2_INST_IRQHandler(void)
{
    uint32_t event_count = 0U;
    DL_TIMER_IIDX pending;

    while ((uint32_t)(pending =
               DL_TimerG_getPendingInterrupt(CAPTURE_ENCODER_M2_INST)) != 0U) {
        if (pending == DL_TIMERG_IIDX_CC0_UP) {
            Encoder_ProcessRightEdge(2U);
            event_count++;
        } else if (pending == DL_TIMERG_IIDX_CC1_UP) {
            Encoder_ProcessRightEdge(1U);
            event_count++;
        }
    }
    if (event_count > g_encoder_right_max_events_per_isr) {
        g_encoder_right_max_events_per_isr = event_count;
    }
}
