#include "bsp_motor.h"

#include "bsp_encoder.h"
#include "ti_msp_dl_config.h"

#define MOTOR_DUTY_PERMILLE_MAX 1000U

static void Motor_WritePin(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void Motor_SetDirectionPins(BSP_MotorId motor, bool in1, bool in2)
{
    if (motor == BSP_MOTOR_LEFT) {
        Motor_WritePin(GPIO_MOTOR_SAFE_PORT,
                       GPIO_MOTOR_SAFE_M1_IN1_PIN, in1);
        Motor_WritePin(GPIO_MOTOR_SAFE_PORT,
                       GPIO_MOTOR_SAFE_M1_IN2_PIN, in2);
    } else {
        Motor_WritePin(GPIO_MOTOR_SAFE_PORT,
                       GPIO_MOTOR_SAFE_M2_IN1_PIN, in1);
        Motor_WritePin(GPIO_MOTOR_SAFE_PORT,
                       GPIO_MOTOR_SAFE_M2_IN2_PIN, in2);
    }
}

static DL_TIMER_CC_INDEX Motor_GetCaptureCompareIndex(BSP_MotorId motor)
{
    return (motor == BSP_MOTOR_LEFT) ? GPIO_PWM_MOTOR12_C0_IDX
                                     : GPIO_PWM_MOTOR12_C1_IDX;
}

static void Motor_SetDuty(BSP_MotorId motor, uint16_t duty_permille)
{
    uint32_t active_counts;
    uint32_t compare;
    uint32_t period_counts = DL_TimerG_getLoadValue(PWM_MOTOR12_INST);

    if (duty_permille > MOTOR_DUTY_PERMILLE_MAX) {
        duty_permille = MOTOR_DUTY_PERMILLE_MAX;
    }

    active_counts = (period_counts * duty_permille) /
                    MOTOR_DUTY_PERMILLE_MAX;
    compare = period_counts - active_counts;
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR12_INST, compare,
                                     Motor_GetCaptureCompareIndex(motor));
}

bool BSP_Motor_Init(void)
{
    BSP_Motor_SetDriverEnabled(false);
    BSP_Motor_SetPowerEnabled(false);
    BSP_Motor_CoastAll();

    DL_TimerG_startCounter(PWM_MOTOR12_INST);

    /* The application starts with zero duty; enable only the M1/M2 bridge. */
    BSP_Motor_SetPowerEnabled(true);
    BSP_Motor_SetDriverEnabled(true);
    return true;
}

void BSP_Motor_SetDriverEnabled(bool enabled)
{
    if (!enabled) {
        BSP_Motor_CoastAll();
    }
    Motor_WritePin(GPIO_MOTOR_SAFE_PORT,
                   GPIO_MOTOR_SAFE_MOTOR12_STBY_PIN, enabled);

}

void BSP_Motor_SetPowerEnabled(bool enabled)
{
    if (!enabled) {
        BSP_Motor_CoastAll();
        DL_GPIO_clearPins(GPIO_MOTOR_SAFE_PORT,
                          GPIO_MOTOR_SAFE_MOTOR12_STBY_PIN);
    }
    Motor_WritePin(GPIO_MOTOR_SAFE_PORT,
                   GPIO_MOTOR_SAFE_MOTOR_VM_EN_PIN, enabled);
}

void BSP_Motor_SetForward(BSP_MotorId motor, uint16_t duty_permille)
{
    if (duty_permille > MOTOR_DUTY_PERMILLE_MAX) {
        duty_permille = MOTOR_DUTY_PERMILLE_MAX;
    }
    BSP_Motor_SetSpeed(motor, (int16_t)duty_permille);
}

void BSP_Motor_SetReverse(BSP_MotorId motor, uint16_t duty_permille)
{
    if (duty_permille > MOTOR_DUTY_PERMILLE_MAX) {
        duty_permille = MOTOR_DUTY_PERMILLE_MAX;
    }
    BSP_Motor_SetSpeed(motor, -(int16_t)duty_permille);
}

void BSP_Motor_SetSpeed(BSP_MotorId motor, int16_t speed_permille)
{
    uint16_t magnitude;

    if (speed_permille > (int16_t)MOTOR_DUTY_PERMILLE_MAX) {
        speed_permille = (int16_t)MOTOR_DUTY_PERMILLE_MAX;
    } else if (speed_permille < -(int16_t)MOTOR_DUTY_PERMILLE_MAX) {
        speed_permille = -(int16_t)MOTOR_DUTY_PERMILLE_MAX;
    }

    if (speed_permille == 0) {
        BSP_Motor_Coast(motor);
        return;
    }

    Motor_SetDuty(motor, 0U);
    /* On this chassis IN1=0/IN2=1 is the physical forward direction for
     * both mirrored motors. Keep the application convention intuitive:
     * positive speed always means vehicle-forward wheel rotation. */
    if (speed_permille > 0) {
        BSP_Encoder_SetExpectedDirection(
            motor == BSP_MOTOR_LEFT, 1);
        Motor_SetDirectionPins(motor, false, true);
        magnitude = (uint16_t)speed_permille;
    } else {
        BSP_Encoder_SetExpectedDirection(
            motor == BSP_MOTOR_LEFT, -1);
        Motor_SetDirectionPins(motor, true, false);
        magnitude = (uint16_t)(-speed_permille);
    }
    Motor_SetDuty(motor, magnitude);
}

void BSP_Motor_Coast(BSP_MotorId motor)
{
    BSP_Encoder_SetExpectedDirection(motor == BSP_MOTOR_LEFT, 0);
    Motor_SetDuty(motor, 0U);
    Motor_SetDirectionPins(motor, false, false);
}

void BSP_Motor_CoastAll(void)
{
    BSP_Motor_Coast(BSP_MOTOR_LEFT);
    BSP_Motor_Coast(BSP_MOTOR_RIGHT);
}
