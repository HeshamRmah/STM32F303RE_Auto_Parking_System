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
	.lcd_data[0].port = GPIOB, .lcd_data[0].pin = GPIO_PIN_10,
	.lcd_data[1].port = GPIOB, .lcd_data[1].pin = GPIO_PIN_11,
	.lcd_data[2].port = GPIOB, .lcd_data[2].pin = GPIO_PIN_12,
	.lcd_data[3].port = GPIOB, .lcd_data[3].pin = GPIO_PIN_13,
	.lcd_rs.port = GPIOB, .lcd_rs.pin = GPIO_PIN_1,
	.lcd_en.port = GPIOB, .lcd_en.pin = GPIO_PIN_2};

ultrasonic_obj_t Front_ultrasonic      = {.htim = &htim1, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t RightFront_ultrasonic = {.htim = &htim2, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t RightRear_ultrasonic  = {.htim = &htim3, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t LeftFront_ultrasonic  = {.htim = &htim4, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t LeftRear_ultrasonic   = {.htim = &htim8, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t Rear_ultrasonic       = {.htim = &htim15,.PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

motor_speed_t high_speed   = {.Frequency = 1000, .Duty_Cycle = 0.8};

motor_speed_t medium_speed = {.Frequency = 1000, .Duty_Cycle = 0.4};

motor_speed_t low_speed    = {.Frequency = 1000, .Duty_Cycle = 0.01};

/*motor_obj_t steering_motor = {.htim = &htim, .PWM_Channel = TIM_CHANNEL_1, .speed.Frequency = 1000, .speed.Duty_Cycle = 0.4};*/

motor_obj_t moving_motor = {.htim = &htim16, .PWM_Channel = TIM_CHANNEL_1, .speed.Frequency = 1000, .speed.Duty_Cycle = 0.4};

bluetooth_obj_t bluetooth = {.huart = &huart4, .Numberofdata = 1};






/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for ultrasonics_read */
osThreadId_t ultrasonics_readHandle;
const osThreadAttr_t ultrasonics_read_attributes = {
  .name = "ultrasonics_read",
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
/* Definitions for bluetooth_recive */
osThreadId_t bluetooth_reciveHandle;
const osThreadAttr_t bluetooth_recive_attributes = {
  .name = "bluetooth_recive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void RTOS_Ultrasonics_Read(void *argument);
void RTOS_Car_Next_Step(void *argument);
void RTOS_Bluetooth_Recive(void *argument);

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

  /* creation of ultrasonics_read */
  ultrasonics_readHandle = osThreadNew(RTOS_Ultrasonics_Read, NULL, &ultrasonics_read_attributes);

  /* creation of car_next_step */
  car_next_stepHandle = osThreadNew(RTOS_Car_Next_Step, NULL, &car_next_step_attributes);

  /* creation of bluetooth_recive */
  bluetooth_reciveHandle = osThreadNew(RTOS_Bluetooth_Recive, NULL, &bluetooth_recive_attributes);

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

	ECU_Bluetooth_ReciveData(&bluetooth);

	ECU_Motor_GeneratePWM(&moving_motor);
	//ECU_Motor_GeneratePWM(&steering_motor);

  /* Infinite loop */
  for(;;)
  {
	  printf("DefaultTask is Running\n");
	  /* Suspend itself */
	  osThreadSuspend(osThreadGetId());
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
	  ECU_Ultrasonic_Read(&Front_ultrasonic);
	  printf("Front_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&RightFront_ultrasonic);
	  printf("RightFront_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&RightRear_ultrasonic);
	  printf("RightRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&LeftFront_ultrasonic);
	  printf("LeftFront_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&LeftRear_ultrasonic);
	  printf("LeftRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&Rear_ultrasonic);
	  printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);

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

/* USER CODE BEGIN Header_RTOS_Bluetooth_Recive */
/**
* @brief Function implementing the bluetooth_recive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_Bluetooth_Recive */
void RTOS_Bluetooth_Recive(void *argument)
{
  /* USER CODE BEGIN RTOS_Bluetooth_Recive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RTOS_Bluetooth_Recive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

