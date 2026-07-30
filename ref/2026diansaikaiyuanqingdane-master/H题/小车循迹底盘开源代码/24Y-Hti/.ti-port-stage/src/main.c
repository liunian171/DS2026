#include "ti_msp_dl_config.h"

#include "bsp_fault.h"
#include "bsp_debug_uart.h"
#include "bsp_encoder.h"
#include "bsp_gyro_uart.h"
#include "bsp_led.h"
#include "bsp_line_uart.h"
#include "bsp_time.h"
#include "cyz_sensor.h"
#include "line_sensor.h"
#include "app_main.h"

#if !H_TASK_APPLICATION
/* 这些符号专门保留给 OpenOCD/GDB 板级自检读取。 */
volatile uint32_t g_board_selftest_heartbeat;
volatile uint32_t g_board_selftest_status;
volatile uint32_t g_board_selftest_i2c_lines;
volatile uint32_t g_board_selftest_encoder_levels;
volatile uint32_t g_board_selftest_encoder_high_seen;
volatile uint32_t g_board_selftest_encoder_low_seen;
volatile uint32_t g_board_selftest_uart_pin_activity;

#define SELFTEST_STATUS_TIME_OK     (1UL << 0)
#define SELFTEST_STATUS_ENCODER_OK  (1UL << 1)
#define SELFTEST_STATUS_GYRO_OK     (1UL << 2)
#define SELFTEST_STATUS_LINE_OK     (1UL << 3)
#define SELFTEST_STATUS_I2C_SDA_HI  (1UL << 4)
#define SELFTEST_STATUS_I2C_SCL_HI  (1UL << 5)
#define SELFTEST_STATUS_MOTOR_SAFE  (1UL << 6)

#define UART_PROBE_GYRO_HIGH_SEEN   (1UL << 0)
#define UART_PROBE_GYRO_LOW_SEEN    (1UL << 1)
#define UART_PROBE_LINE_HIGH_SEEN   (1UL << 2)
#define UART_PROBE_LINE_LOW_SEEN    (1UL << 3)

static void BoardSelfTest_ProbeUARTInputs(void)
{
    uint32_t sample;

    DL_GPIO_initDigitalInputFeatures(GPIO_UART_GYRO_IOMUX_RX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_UART_LINE_IOMUX_RX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    g_board_selftest_uart_pin_activity = 0U;
    for (sample = 0U; sample < 1000000U; ++sample) {
        if ((DL_GPIO_readPins(GPIO_UART_GYRO_RX_PORT,
                 GPIO_UART_GYRO_RX_PIN) & GPIO_UART_GYRO_RX_PIN) != 0U) {
            g_board_selftest_uart_pin_activity |= UART_PROBE_GYRO_HIGH_SEEN;
        } else {
            g_board_selftest_uart_pin_activity |= UART_PROBE_GYRO_LOW_SEEN;
        }
        if ((DL_GPIO_readPins(GPIO_UART_LINE_RX_PORT,
                 GPIO_UART_LINE_RX_PIN) & GPIO_UART_LINE_RX_PIN) != 0U) {
            g_board_selftest_uart_pin_activity |= UART_PROBE_LINE_HIGH_SEEN;
        } else {
            g_board_selftest_uart_pin_activity |= UART_PROBE_LINE_LOW_SEEN;
        }
    }

    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_GYRO_IOMUX_RX, GPIO_UART_GYRO_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_LINE_IOMUX_RX, GPIO_UART_LINE_IOMUX_RX_FUNC);
}

static void BoardSelfTest_UpdateInputs(void)
{
    const uint32_t encoder_mask = GPIO_CAPTURE_ENCODER_M1_C0_PIN |
        GPIO_CAPTURE_ENCODER_M1_C1_PIN | GPIO_CAPTURE_ENCODER_M2_C0_PIN |
        GPIO_CAPTURE_ENCODER_M2_C1_PIN;
    uint32_t i2c_pins = DL_GPIO_readPins(GPIO_I2C_EXP_SDA_PORT,
        GPIO_I2C_EXP_SDA_PIN | GPIO_I2C_EXP_SCL_PIN);
    uint32_t encoder_pins = DL_GPIO_readPins(
        GPIO_CAPTURE_ENCODER_M1_C0_PORT, encoder_mask);
    uint32_t motor_pins = DL_GPIO_readPins(GPIO_MOTOR_SAFE_PORT,
        GPIO_MOTOR_SAFE_M1_IN1_PIN | GPIO_MOTOR_SAFE_M1_IN2_PIN |
        GPIO_MOTOR_SAFE_M2_IN1_PIN | GPIO_MOTOR_SAFE_M2_IN2_PIN |
        GPIO_MOTOR_SAFE_MOTOR12_STBY_PIN |
        GPIO_MOTOR_SAFE_MOTOR_VM_EN_PIN);

    g_board_selftest_i2c_lines = i2c_pins;
    g_board_selftest_encoder_levels = encoder_pins;
    g_board_selftest_encoder_high_seen |= encoder_pins;
    g_board_selftest_encoder_low_seen |= (~encoder_pins) & encoder_mask;
    g_board_selftest_status &= ~(SELFTEST_STATUS_I2C_SDA_HI |
        SELFTEST_STATUS_I2C_SCL_HI | SELFTEST_STATUS_MOTOR_SAFE);
    if ((i2c_pins & GPIO_I2C_EXP_SDA_PIN) != 0U) {
        g_board_selftest_status |= SELFTEST_STATUS_I2C_SDA_HI;
    }
    if ((i2c_pins & GPIO_I2C_EXP_SCL_PIN) != 0U) {
        g_board_selftest_status |= SELFTEST_STATUS_I2C_SCL_HI;
    }
    if (motor_pins == 0U) {
        g_board_selftest_status |= SELFTEST_STATUS_MOTOR_SAFE;
    }
}
#endif

int main(void)
{
    SYSCFG_DL_init();
    BSP_LED_Set(false);
    if (!BSP_Time_Init()) {
        BSP_Fault();
    }
#if H_TASK_APPLICATION
    App_Init();
    while (1) {
        App_Run();
    }
#else
    BoardSelfTest_ProbeUARTInputs();
    g_board_selftest_status = SELFTEST_STATUS_TIME_OK;
    /*
     * 安全板级自检：电机和舵机均不启动。UART/编码器仅接收和计数，
     * 结果可通过 OpenOCD 读取，即使没有连接调试串口也能验收。
     */
    if (BSP_Encoder_Init()) {
        g_board_selftest_status |= SELFTEST_STATUS_ENCODER_OK;
    }
    if (BSP_GyroUART_Init()) {
        g_board_selftest_status |= SELFTEST_STATUS_GYRO_OK;
    }
    if (!CYZ_Sensor_Init()) {
        BSP_Fault();
    }
    if (BSP_LineUART_Init()) {
        g_board_selftest_status |= SELFTEST_STATUS_LINE_OK;
    }
    if (!LineSensor_Init()) {
        BSP_Fault();
    }
    BSP_DebugUART_Write("\r\nMSPM0G3507 board self-test ready\r\n");
    BoardSelfTest_UpdateInputs();

    while (1) {
        uint32_t sample;

        for (sample = 0U; sample < 250U; ++sample) {
            BSP_Time_DelayMs(1U);
            CYZ_Sensor_Process(BSP_Time_GetMs());
            LineSensor_Process(BSP_Time_GetMs());
            BoardSelfTest_UpdateInputs();
        }
        BSP_LED_Toggle();
        ++g_board_selftest_heartbeat;
    }
#endif
}
