/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver/motor_bridge.h"
#include "driver/pwm.h"               // 需要 PWM_Handle 的完整定义
#include "driver/usergpio.h"
#include "driver/usergpio_platform.h"
#include "driver/uart.h"
#include "driver/uart_platform_ops.h"
#include "driver/uart_cmd_parser.h"
#include "common/ringbuf.h"
#include "useri2c_ops.h"
#include "imu_bridge.h"
#include "imu_uart_handler.h"
#include "oled_bridge.h"
#include "encoder.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---------- UART 调试串口 ---------- */
UART_Handle    uart_debug = {
    .huart      = &huart7,
    .ops        = &uart_platform_ops_stm32,
};

static RingBuffer g_ringbuf_debug;


I2C_SoftwareContext g_software_i2c = {
    .scl_pin = { .hgpio_port = GPIOF, .gpio_pin = 0,
                 .ops = &usergpio_platform_ops_stm32 },
    .sda_pin = { .hgpio_port = GPIOF, .gpio_pin = 1,
                 .ops = &usergpio_platform_ops_stm32 },
    .timer_handle = &htim6,
    .macro_state = I2C_MACRO_IDLE,
    .micro_state = I2C_MICRO_IDLE,
};

I2C_Handle g_i2c_dev = {
    .ops         = &i2c_software_platform_ops_stm32,
    .i2c_context = &g_software_i2c,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* ---- I2C 降速：ARR=899 → 10μs/tick (~50kHz) ---- */
  htim6.Init.Period = 899;
  HAL_TIM_Base_Init(&htim6);

  /* ---------- 启动 UART（回显测试） ---------- */
  ringbuf_init(&g_ringbuf_debug);
  uart_cmd_parser_init(&uart_debug, &g_ringbuf_debug);
  uart_receive_IT(&uart_debug, &uart_debug.rx_byte);

  /* 启动时发一条消息，串口助手看到说明 TX 通路正常 */
  uart_send(&uart_debug, (uint8_t *)"UART7 Ready!\r\n", 14);

  /* ---- OLED + 编码器初始化 ---- */
  oled_bridge_init();
  oled_bridge_show_string(0, 0, "ENC TEST");

  /* ---- 编码器测试 ---- */
  Encoder_Handle henc;
  henc.htim  = &htim2;
  henc.ops   = encoder_platform_get_ops();
  henc.ppr   = 1320;          // 填入你的编码器 PPR
  henc.position = 0;
  encoder_start(&henc);

  /* TODO: 调试指示灯
     心跳灯 — GPIO 控制, 500ms 翻转 (Systick 里或主循环定时), 证明主循环活着
     通信灯 — 收/发时翻转 GPIO, 证明串口通路正常
     两个 GPIO + 两个限流电阻即可, 调试时最保值的投入 */

#if 0   /* 电机部分暂时禁用，先验证串口 */
  // 电机测试：50% 占空比正转
  pwm_set_freq(&pwm_tim5_ch4, 20000);              // 电机 PWM 频率 20kHz
  pwm_start(&pwm_tim5_ch4);                        // 启动 PWM 输出

  motor_bridge_init(0, &pwm_tim5_ch4,
                    &motor_in1, &motor_in2, &motor_stby,
                    200.0f, 33.1f);

  motor_bridge_set_speed_rpm(0, 100);              // 50% 速度（200×50%=100 RPM）
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- 串口命令解析 ---- */
    uart_cmd_parser_tick();

    /* ---- 显示编码器 ---- */
    {
        static uint32_t t = 0;
        if (HAL_GetTick() - t >= 500) {
            t = HAL_GetTick();

            int32_t cnt = encoder_get_count(&henc);
            char b[17];
            snprintf(b, 17, "ENC:%d", (int)cnt);
            oled_bridge_show_string(1, 0, b);
        }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
