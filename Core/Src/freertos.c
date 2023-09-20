/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
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
/* USER CODE BEGIN Variables */
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



/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for ultrasonics_rea */
osThreadId_t ultrasonics_reaHandle;
const osThreadAttr_t ultrasonics_rea_attributes = {
  .name = "ultrasonics_rea",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for car_next_step */
osThreadId_t car_next_stepHandle;
const osThreadAttr_t car_next_step_attributes = {
  .name = "car_next_step",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void RTOS_Ultrasonics_Read(void *argument);
void RTOS_Car_Next_Step(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);

  /* creation of ultrasonics_rea */
  ultrasonics_reaHandle = osThreadNew(RTOS_Ultrasonics_Read, NULL, &ultrasonics_rea_attributes);

  /* creation of car_next_step */
  car_next_stepHandle = osThreadNew(RTOS_Car_Next_Step, NULL, &car_next_step_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTask */
void DefaultTask(void *argument)
{
  /* USER CODE BEGIN DefaultTask */
	printf("DefaultTask is Running\n");
	ECU_Bluetooth_ReciveData(&bluetooth);

	ECU_Motor_GeneratePWM(&moving_motor);

	//ECU_Motor_GeneratePWM(&steering_motor);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END DefaultTask */
}

/* USER CODE BEGIN Header_RTOS_Ultrasonics_Read */
/**
* @brief Function implementing the ultrasonics_rea thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_Ultrasonics_Read */
void RTOS_Ultrasonics_Read(void *argument)
{
  /* USER CODE BEGIN RTOS_Ultrasonics_Read */
  /* Infinite loop */
  for(;;)
  {
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
    osDelay(200);
  }
  /* USER CODE END RTOS_Ultrasonics_Read */
}

/* USER CODE BEGIN Header_RTOS_Car_Next_Step */
/**
* @brief Function implementing the car_next_step thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_Car_Next_Step */
void RTOS_Car_Next_Step(void *argument)
{
  /* USER CODE BEGIN RTOS_Car_Next_Step */
  /* Infinite loop */
  for(;;)
  {
	  printf("RTOS_Car_Next_Step is Running\n");
	  ECU_Motor_NextStep(&moving_motor, &Bluetooth_RX_Data);

	  osDelay(500);
  }
  /* USER CODE END RTOS_Car_Next_Step */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

