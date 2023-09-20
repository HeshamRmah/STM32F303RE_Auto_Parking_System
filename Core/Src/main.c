/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "../../ECU/ECU_std_types.h"
#include "../../MCAL/GPIO/mcal_gpio.h"
#include "../../Drivers/ECU/Motor/ecu_motor.h"
#include "../../Drivers/ECU/LCD/hal_lcd.h"
#include "../../Drivers/ECU/Ultrasonic/ultrasonic.h"
#include "../../Drivers/ECU/Bluetooth/bluetooth.h"


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
chr_lcd_4bit_t lcd = {
	.lcd_data[0].port = GPIOB,
	.lcd_data[0].pin = GPIO_PIN_10,

	.lcd_data[1].port = GPIOB,
	.lcd_data[1].pin = GPIO_PIN_11,

	.lcd_data[2].port = GPIOB,
	.lcd_data[2].pin = GPIO_PIN_12,

	.lcd_data[3].port = GPIOB,
	.lcd_data[3].pin = GPIO_PIN_13,

	.lcd_rs.port = GPIOB,
	.lcd_rs.pin = GPIO_PIN_1,

	.lcd_en.port = GPIOB,
	.lcd_en.pin = GPIO_PIN_2,
};

ultrasonic_obj_t ultrasonic_1 = {
		.htim = &htim1,
		.PWM_Channel = TIM_CHANNEL_1,
		.IC_Channel = TIM_CHANNEL_2,
};

ultrasonic_obj_t ultrasonic_2 = {
		.htim = &htim2,
		.PWM_Channel = TIM_CHANNEL_1,
		.IC_Channel = TIM_CHANNEL_2
};

ultrasonic_obj_t ultrasonic_3 = {
		.htim = &htim3,
		.PWM_Channel = TIM_CHANNEL_1,
		.IC_Channel = TIM_CHANNEL_2
};

ultrasonic_obj_t ultrasonic_4 = {
		.htim = &htim4,
		.PWM_Channel = TIM_CHANNEL_1,
		.IC_Channel = TIM_CHANNEL_2
};

ultrasonic_obj_t ultrasonic_5 = {
		.htim = &htim8,
		.PWM_Channel = TIM_CHANNEL_1,
		.IC_Channel = TIM_CHANNEL_2
};

ultrasonic_obj_t ultrasonic_6 = {
		.htim = &htim15,
		.PWM_Channel = TIM_CHANNEL_1,
		.IC_Channel = TIM_CHANNEL_2
};

motor_obj_t steering_motor = {
		.htim = &htim16,
		.PWM_Channel = TIM_CHANNEL_1,
		.speed.Frequency = 100,
		.speed.Duty_Cycle = 0.5,
};

motor_obj_t moving_motor = {
		.htim = &htim16,
		.PWM_Channel = TIM_CHANNEL_1,
		.speed.Frequency = 1000,
		.speed.Duty_Cycle = 0.5,
};

bluetooth_obj_t bluetooth = {
		.huart = &huart4,
		.Numberofdata = 1,
};


motor_obj_t test1 = {
		.htim = &htim1,
		.PWM_Channel = TIM_CHANNEL_1,
		.speed.Frequency = 1000,
		.speed.Duty_Cycle = 0.01,
};

motor_obj_t test2 = {
		.htim = &htim2,
		.PWM_Channel = TIM_CHANNEL_1,
		.speed.Frequency = 100,
		.speed.Duty_Cycle = 0.25,
};

motor_obj_t test3 = {
		.htim = &htim3,
		.PWM_Channel = TIM_CHANNEL_1,
		.speed.Frequency = 100,
		.speed.Duty_Cycle = 0.75,
};

motor_obj_t test4 = {
		.htim = &htim4,
		.PWM_Channel = TIM_CHANNEL_1,
		.speed.Frequency = 1000,
		.speed.Duty_Cycle = 0.5,
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  //ECU_Motor_GeneratePWM(&moving_motor);

//  ECU_Motor_GeneratePWM(&steering_motor);
//	ECU_Motor_GeneratePWM(&test1);
//	ECU_Motor_GeneratePWM(&test2);
//	ECU_Motor_GeneratePWM(&test3);
//	ECU_Motor_GeneratePWM(&test4);

	//ECU_Bluetooth_ReciveData(&bluetooth);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //printf("The value is %c\n",Bluetooth_RX_Data);

	  //ECU_Motor_NextStep(&moving_motor, &Bluetooth_RX_Data);
//	  ECU_Motor_MoveRight(&steering_motor);
//	  HAL_Delay(5000);
//
//	  ECU_Motor_Stop(&steering_motor);
//	  HAL_Delay(500);
//
//	  ECU_Motor_MoveLeft(&steering_motor);
//	  HAL_Delay(5000);

	  ECU_Ultrasonic_Read(&ultrasonic_1);
	  printf("ultrasonic 1 value is %lu\n",ultrasonic_Distance_Values[0]);
	  //HAL_Delay(10);

	  ECU_Ultrasonic_Read(&ultrasonic_2);
	  printf("ultrasonic 2 value is %lu\n",ultrasonic_Distance_Values[1]);
	  //HAL_Delay(10);

	  ECU_Ultrasonic_Read(&ultrasonic_3);
	  printf("ultrasonic 3 value is %lu\n",ultrasonic_Distance_Values[2]);
	  //HAL_Delay(10);

	  ECU_Ultrasonic_Read(&ultrasonic_4);
	  printf("ultrasonic 4 value is %lu\n",ultrasonic_Distance_Values[3]);
	  //HAL_Delay(10);

	  ECU_Ultrasonic_Read(&ultrasonic_5);
	  printf("ultrasonic 5 value is %lu\n",ultrasonic_Distance_Values[4]);
	  //HAL_Delay(10);

	  ECU_Ultrasonic_Read(&ultrasonic_6);
	  printf("ultrasonic 6 value is %lu\n",ultrasonic_Distance_Values[5]);
	  //HAL_Delay(10);

	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  //HAL_Delay(500);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
