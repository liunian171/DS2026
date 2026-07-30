#include "zdt_stepper.h"

#include "bsp_stepper_uart.h"

#define ZDT_FRAME_TAIL 0x6BU
#define ZDT_FUNCTION_ENABLE 0xF3U
#define ZDT_FUNCTION_POSITION 0xFDU
#define ZDT_FUNCTION_STOP 0xFEU
#define ZDT_FUNCTION_READ_POSITION 0x36U
#define ZDT_MAX_SPEED_RPM 5000U
#define ZDT_RX_FRAME_MAX_LENGTH 8U
#define ZDT_POSITION_UNITS_PER_REVOLUTION 65536LL

static uint8_t zdt_address;
static uint32_t zdt_pulses_per_revolution;
static uint8_t zdt_rx_frame[ZDT_RX_FRAME_MAX_LENGTH];
static uint8_t zdt_rx_index;
static uint8_t zdt_rx_expected_length;

volatile uint32_t g_zdt_stepper_initialized;
volatile uint32_t g_zdt_stepper_tx_command_count;
volatile uint32_t g_zdt_stepper_rx_frame_count;
volatile uint32_t g_zdt_stepper_rx_error_count;
volatile uint32_t g_zdt_stepper_position_valid;
volatile uint32_t g_zdt_stepper_last_function;
volatile uint32_t g_zdt_stepper_last_status;
volatile int32_t g_zdt_stepper_position_raw;
volatile int32_t g_zdt_stepper_position_pulses;
volatile int32_t g_zdt_stepper_angle_mdeg;

static uint32_t ZDT_AbsInt32ToUint32(int32_t value)
{
    if (value >= 0)
    {
        return (uint32_t)value;
    }
    return (uint32_t)(-(int64_t)value);
}

static int32_t ZDT_ClampInt64ToInt32(int64_t value)
{
    if (value > INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value < INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static bool ZDT_Send(const uint8_t *frame, uint16_t length)
{
    if (!g_zdt_stepper_initialized ||
        !BSP_StepperUART_Write(frame, length))
    {
        return false;
    }
    g_zdt_stepper_tx_command_count++;
    return true;
}

static bool ZDT_MovePulses(int32_t pulses,
                           uint16_t speed_rpm,
                           uint8_t acceleration,
                           bool absolute)
{
    uint32_t magnitude = ZDT_AbsInt32ToUint32(pulses);
    uint8_t frame[13];

    if (speed_rpm > ZDT_MAX_SPEED_RPM)
    {
        speed_rpm = ZDT_MAX_SPEED_RPM;
    }
    frame[0] = zdt_address;
    frame[1] = ZDT_FUNCTION_POSITION;
    frame[2] = (pulses < 0) ? 1U : 0U;
    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(magnitude >> 24);
    frame[7] = (uint8_t)(magnitude >> 16);
    frame[8] = (uint8_t)(magnitude >> 8);
    frame[9] = (uint8_t)magnitude;
    frame[10] = absolute ? 1U : 0U;
    frame[11] = 0U;
    frame[12] = ZDT_FRAME_TAIL;
    return ZDT_Send(frame, sizeof(frame));
}

static void ZDT_ResetParser(void)
{
    zdt_rx_index = 0U;
    zdt_rx_expected_length = 0U;
}

static void ZDT_PublishFrame(void)
{
    uint8_t function = zdt_rx_frame[1];

    if (zdt_rx_frame[zdt_rx_expected_length - 1U] != ZDT_FRAME_TAIL)
    {
        g_zdt_stepper_rx_error_count++;
        return;
    }
    g_zdt_stepper_last_function = function;
    if (function == ZDT_FUNCTION_READ_POSITION)
    {
        uint32_t magnitude =
            ((uint32_t)zdt_rx_frame[3] << 24) |
            ((uint32_t)zdt_rx_frame[4] << 16) |
            ((uint32_t)zdt_rx_frame[5] << 8) |
            (uint32_t)zdt_rx_frame[6];
        int64_t signed_raw_position = (int64_t)magnitude;
        int64_t position_pulses;
        int64_t angle_mdeg;

        if (zdt_rx_frame[2] != 0U)
        {
            signed_raw_position = -signed_raw_position;
        }
        position_pulses =
            (signed_raw_position * zdt_pulses_per_revolution) /
            ZDT_POSITION_UNITS_PER_REVOLUTION;
        angle_mdeg = (signed_raw_position * 360000LL) /
                     ZDT_POSITION_UNITS_PER_REVOLUTION;
        g_zdt_stepper_position_raw =
            ZDT_ClampInt64ToInt32(signed_raw_position);
        g_zdt_stepper_position_pulses =
            ZDT_ClampInt64ToInt32(position_pulses);
        g_zdt_stepper_angle_mdeg =
            ZDT_ClampInt64ToInt32(angle_mdeg);
        g_zdt_stepper_position_valid = 1U;
        g_zdt_stepper_last_status = 0U;
    }
    else
    {
        g_zdt_stepper_last_status = zdt_rx_frame[2];
    }
    g_zdt_stepper_rx_frame_count++;
}

static void ZDT_InputByte(uint8_t byte)
{
    if (zdt_rx_index == 0U)
    {
        if (byte != zdt_address)
        {
            return;
        }
        zdt_rx_frame[zdt_rx_index++] = byte;
        return;
    }
    if (zdt_rx_index >= ZDT_RX_FRAME_MAX_LENGTH)
    {
        g_zdt_stepper_rx_error_count++;
        ZDT_ResetParser();
        return;
    }
    zdt_rx_frame[zdt_rx_index++] = byte;
    if (zdt_rx_index == 2U)
    {
        zdt_rx_expected_length =
            (byte == ZDT_FUNCTION_READ_POSITION) ? 8U : 4U;
    }
    if ((zdt_rx_expected_length > 0U) &&
        (zdt_rx_index >= zdt_rx_expected_length))
    {
        ZDT_PublishFrame();
        ZDT_ResetParser();
    }
}

bool ZDT_Stepper_Init(uint8_t address, uint32_t pulses_per_revolution)
{
    if ((address == 0U) || (pulses_per_revolution == 0U))
    {
        return false;
    }
    zdt_address = address;
    zdt_pulses_per_revolution = pulses_per_revolution;
    ZDT_ResetParser();
    g_zdt_stepper_tx_command_count = 0U;
    g_zdt_stepper_rx_frame_count = 0U;
    g_zdt_stepper_rx_error_count = 0U;
    g_zdt_stepper_position_valid = 0U;
    g_zdt_stepper_last_function = 0U;
    g_zdt_stepper_last_status = 0U;
    g_zdt_stepper_position_raw = 0;
    g_zdt_stepper_position_pulses = 0;
    g_zdt_stepper_angle_mdeg = 0;
    g_zdt_stepper_initialized = 1U;
    return true;
}

void ZDT_Stepper_Process(void)
{
    uint8_t byte;

    while (BSP_StepperUART_ReadByte(&byte))
    {
        ZDT_InputByte(byte);
    }
}

bool ZDT_Stepper_Enable(bool enable)
{
    const uint8_t frame[6] = {
        zdt_address, ZDT_FUNCTION_ENABLE, 0xABU,
        enable ? 1U : 0U, 0U, ZDT_FRAME_TAIL
    };
    return ZDT_Send(frame, sizeof(frame));
}

bool ZDT_Stepper_Stop(void)
{
    const uint8_t frame[5] = {
        zdt_address, ZDT_FUNCTION_STOP, 0x98U, 0U, ZDT_FRAME_TAIL
    };
    return ZDT_Send(frame, sizeof(frame));
}

bool ZDT_Stepper_MoveAbsolutePulses(int32_t position_pulses,
                                    uint16_t speed_rpm,
                                    uint8_t acceleration)
{
    return ZDT_MovePulses(position_pulses, speed_rpm,
                          acceleration, true);
}

bool ZDT_Stepper_MoveRelativePulses(int32_t delta_pulses,
                                    uint16_t speed_rpm,
                                    uint8_t acceleration)
{
    return ZDT_MovePulses(delta_pulses, speed_rpm,
                          acceleration, false);
}

bool ZDT_Stepper_SetAngleMdeg(int32_t angle_mdeg,
                              uint16_t speed_rpm,
                              uint8_t acceleration)
{
    int64_t magnitude = (int64_t)ZDT_AbsInt32ToUint32(angle_mdeg);
    int64_t pulses = (magnitude * zdt_pulses_per_revolution +
                      180000LL) / 360000LL;

    if (pulses > INT32_MAX)
    {
        return false;
    }
    if (angle_mdeg < 0)
    {
        pulses = -pulses;
    }
    return ZDT_Stepper_MoveAbsolutePulses(
        (int32_t)pulses, speed_rpm, acceleration);
}

bool ZDT_Stepper_RequestPosition(void)
{
    const uint8_t frame[3] = {
        zdt_address, ZDT_FUNCTION_READ_POSITION, ZDT_FRAME_TAIL
    };
    return ZDT_Send(frame, sizeof(frame));
}

int32_t ZDT_Stepper_GetPositionPulses(void)
{
    return g_zdt_stepper_position_pulses;
}

int32_t ZDT_Stepper_GetAngleMdeg(void)
{
    return g_zdt_stepper_angle_mdeg;
}
