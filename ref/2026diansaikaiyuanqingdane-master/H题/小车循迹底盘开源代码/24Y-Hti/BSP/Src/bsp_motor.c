#include "bsp_motor.h"

#include "main.h"
#include "tim.h"

#define MOTOR_DUTY_PERMILLE_MAX 1000U

static uint32_t Motor_GetChannel(BSP_MotorId motor)
{
    /* Chassis mapping confirmed by the stage 2 wheel test:
       TB6612 B drives the left wheel and TB6612 A drives the right wheel. */
    return (motor == BSP_MOTOR_LEFT) ? TIM_CHANNEL_2 : TIM_CHANNEL_1;
}

static void Motor_SetDirectionPins(BSP_MotorId motor,
                                   GPIO_PinState in1,
                                   GPIO_PinState in2)
{
    if (motor == BSP_MOTOR_RIGHT)
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_Port, MOTOR_AIN1_Pin, in1);
        HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_Port, MOTOR_AIN2_Pin, in2);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_BIN1_GPIO_Port, MOTOR_BIN1_Pin, in1);
        HAL_GPIO_WritePin(MOTOR_BIN2_GPIO_Port, MOTOR_BIN2_Pin, in2);
    }
}

static void Motor_SetDuty(BSP_MotorId motor, uint16_t duty_permille)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t compare;

    if (duty_permille > MOTOR_DUTY_PERMILLE_MAX)
    {
        duty_permille = MOTOR_DUTY_PERMILLE_MAX;
    }

    compare = (period * (uint32_t)duty_permille) / MOTOR_DUTY_PERMILLE_MAX;
    __HAL_TIM_SET_COMPARE(&htim3, Motor_GetChannel(motor), compare);
}

bool BSP_Motor_Init(void)
{
    BSP_Motor_CoastAll();

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        BSP_Motor_CoastAll();
        return false;
    }

    return true;
}

void BSP_Motor_SetForward(BSP_MotorId motor, uint16_t duty_permille)
{
    if (duty_permille > MOTOR_DUTY_PERMILLE_MAX)
    {
        duty_permille = MOTOR_DUTY_PERMILLE_MAX;
    }
    BSP_Motor_SetSpeed(motor, (int16_t)duty_permille);
}

void BSP_Motor_SetReverse(BSP_MotorId motor, uint16_t duty_permille)
{
    if (duty_permille > MOTOR_DUTY_PERMILLE_MAX)
    {
        duty_permille = MOTOR_DUTY_PERMILLE_MAX;
    }
    BSP_Motor_SetSpeed(motor, -(int16_t)duty_permille);
}

void BSP_Motor_SetSpeed(BSP_MotorId motor, int16_t speed_permille)
{
    uint16_t magnitude;

    if (speed_permille > (int16_t)MOTOR_DUTY_PERMILLE_MAX)
    {
        speed_permille = (int16_t)MOTOR_DUTY_PERMILLE_MAX;
    }
    else if (speed_permille < -(int16_t)MOTOR_DUTY_PERMILLE_MAX)
    {
        speed_permille = -(int16_t)MOTOR_DUTY_PERMILLE_MAX;
    }

    if (speed_permille == 0)
    {
        BSP_Motor_Coast(motor);
        return;
    }

    Motor_SetDuty(motor, 0U);
    if (speed_permille > 0)
    {
        /* Both replacement wheel motors are wired opposite to the original
           motors. Keep positive software speed as physical vehicle-forward. */
        Motor_SetDirectionPins(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
        magnitude = (uint16_t)speed_permille;
    }
    else
    {
        Motor_SetDirectionPins(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
        magnitude = (uint16_t)(-speed_permille);
    }
    Motor_SetDuty(motor, magnitude);
}

void BSP_Motor_Coast(BSP_MotorId motor)
{
    Motor_SetDuty(motor, 0U);
    Motor_SetDirectionPins(motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void BSP_Motor_CoastAll(void)
{
    BSP_Motor_Coast(BSP_MOTOR_LEFT);
    BSP_Motor_Coast(BSP_MOTOR_RIGHT);
}
