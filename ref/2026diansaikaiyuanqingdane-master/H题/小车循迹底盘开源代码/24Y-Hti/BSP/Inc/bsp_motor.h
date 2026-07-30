#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_MOTOR_LEFT = 0,
    BSP_MOTOR_RIGHT
} BSP_MotorId;

bool BSP_Motor_Init(void);
void BSP_Motor_SetSpeed(BSP_MotorId motor, int16_t speed_permille);
void BSP_Motor_SetForward(BSP_MotorId motor, uint16_t duty_permille);
void BSP_Motor_SetReverse(BSP_MotorId motor, uint16_t duty_permille);
void BSP_Motor_Coast(BSP_MotorId motor);
void BSP_Motor_CoastAll(void);

#endif /* BSP_MOTOR_H */
