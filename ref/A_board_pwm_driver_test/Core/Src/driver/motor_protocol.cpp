#include "motor_protocol.h"
#include "usergpio.h"
#include <stdint.h>
TB6612MotorProtocol::TB6612MotorProtocol(PWM_Handle *pwm_handle,
                                         UserGPIO_Handle *ain1_gpio,
                                         UserGPIO_Handle *ain2_gpio,
                                         UserGPIO_Handle *stby_gpio)
    : hpwm(pwm_handle), hain1_gpio(ain1_gpio), hain2_gpio(ain2_gpio),
      hstby_gpio(stby_gpio) {}

TB6612MotorProtocol::~TB6612MotorProtocol() {}

// typedef enum {
//     TB6612CH_A,
//     TB6612CH_B,
// }TB6612CH;

void TB6612MotorProtocol::set_speed_rate_0E3(int16_t rpm_rate_0E3) 
{
//根据正负设计两io口电平
//根据rate_0E3设置pwm的占空比,arr固定
  if (rpm_rate_0E3 > 0) 
  {
  usergpio_write(hstby_gpio, 1);
    usergpio_write(hain1_gpio, 1);
    usergpio_write(hain2_gpio, 0);
    pwm_set_duty_0E3(hpwm, rpm_rate_0E3);
  }
  else if (rpm_rate_0E3 == 0)
  {
    stop();
  }

  else 
  {
  usergpio_write(hstby_gpio, 1);
    usergpio_write(hain1_gpio, 0);
    usergpio_write(hain2_gpio, 1);
    pwm_set_duty_0E3(hpwm, -rpm_rate_0E3);
  }
}

void TB6612MotorProtocol::brake() 
{
  usergpio_write(hstby_gpio, 1);
  usergpio_write(hain1_gpio, 1);
  usergpio_write(hain2_gpio, 1);
  pwm_set_duty_0E3(hpwm, 0);
}
void TB6612MotorProtocol::stop()
{
  usergpio_write(hstby_gpio, 0);
  usergpio_write(hain1_gpio, 0);
  usergpio_write(hain2_gpio, 0);
}