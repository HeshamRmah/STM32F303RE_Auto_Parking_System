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
#define CAR_CONTROL_BIT   (1 << 0)
#define PARKING_SIDE_BIT  (1 << 1)
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

/*motor_obj_t steering_motor = {.htim = &htim, .PWM_Channel = TIM_CHANNEL_1, .speed.Frequency = 1000, .speed.Duty_Cycle = 0.4};*/

motor_obj_t moving_motor = {.htim = &htim16, .PWM_Channel = TIM_CHANNEL_1, .speed.Frequency = 1000, .speed.Duty_Cycle = 0.65};

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
/* Definitions for Automatic_Parking */
osThreadId_t Automatic_ParkingHandle;
const osThreadAttr_t Automatic_Parking_attributes = {
  .name = "Automatic_Parking",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for Car_Conrol_Mode */
osMutexId_t Car_Conrol_ModeHandle;
const osMutexAttr_t Car_Conrol_Mode_attributes = {
  .name = "Car_Conrol_Mode"
};
/* Definitions for Car_mode */
osEventFlagsId_t Car_modeHandle;
const osEventFlagsAttr_t Car_mode_attributes = {
  .name = "Car_mode"
};
/* Definitions for Parking_side */
osEventFlagsId_t Parking_sideHandle;
const osEventFlagsAttr_t Parking_side_attributes = {
  .name = "Parking_side"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void RTOS_Ultrasonics_Read(void *argument);
void RTOS_Car_Next_Step(void *argument);
void RTOS_Bluetooth_Recive(void *argument);
void RTOS_Automatic_Parking(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Car_Conrol_Mode */
  Car_Conrol_ModeHandle = osMutexNew(&Car_Conrol_Mode_attributes);

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

  /* creation of Automatic_Parking */
  Automatic_ParkingHandle = osThreadNew(RTOS_Automatic_Parking, NULL, &Automatic_Parking_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of Car_mode */
  Car_modeHandle = osEventFlagsNew(&Car_mode_attributes);

  /* creation of Parking_side */
  Parking_sideHandle = osEventFlagsNew(&Parking_side_attributes);

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
	  //printf("Front_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&RightFront_ultrasonic);
	  //printf("RightFront_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&RightRear_ultrasonic);
	  //printf("RightRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&LeftFront_ultrasonic);
	  //printf("LeftFront_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&LeftRear_ultrasonic);
	  //printf("LeftRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);

	  ECU_Ultrasonic_Read(&Rear_ultrasonic);
	  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);

    osDelay(90);
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
	  //printf("RTOS_Car_Next_Step is Running\n");
	  if( (Bluetooth_RX_Data == AUTO_PARK_RIGHT) ){
		  //osMutexAcquire(Car_Conrol_ModeHandle, 10U);
		  osEventFlagsSet(Car_modeHandle, CAR_CONTROL_BIT);
		  osEventFlagsClear(Car_modeHandle, PARKING_SIDE_BIT);
	  }
	  else if( (Bluetooth_RX_Data == AUTO_PARK_LEFT) ){
		  osEventFlagsSet(Car_modeHandle, CAR_CONTROL_BIT);
		  osEventFlagsSet(Car_modeHandle, PARKING_SIDE_BIT);
	  }
	  else{/* DO NOTHING */}

	  if( (osEventFlagsGet(Car_modeHandle) & CAR_CONTROL_BIT) == STD_IDLE){
		  printf("Mobile Control Mode\n");
		  ECU_Motor_NextStep(&moving_motor, &Bluetooth_RX_Data);
	  }
	  else{/* DO NOTHING */}

	  osDelay(100);
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
    osDelay(100000);
  }
  /* USER CODE END RTOS_Bluetooth_Recive */
}

/* USER CODE BEGIN Header_RTOS_Automatic_Parking */
/**
* @brief Function implementing the Automatic_Parking thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_Automatic_Parking */
void RTOS_Automatic_Parking(void *argument)
{
  /* USER CODE BEGIN RTOS_Automatic_Parking */
  /* Infinite loop */
  for(;;)
  {
	  if( (Bluetooth_RX_Data == AUTO_PARK_OFF) ){
		  //osMutexRelease(Car_Conrol_ModeHandle);
		  osEventFlagsClear(Car_modeHandle, CAR_CONTROL_BIT);
	  }
	  else{/* DO NOTHING */}

	  if( (osEventFlagsGet(Car_modeHandle) & CAR_CONTROL_BIT) == STD_ACTIVE){
		  printf("------------------------------------------------Automatic Parking Mode\n");
		  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);
		  /* Select Right Side to Park */
		  if((osEventFlagsGet(Car_modeHandle) & PARKING_SIDE_BIT) == (STD_IDLE << PARKING_SIDE_BIT)){
			  printf("Park Right Side\n");
			  while((ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX] < 16) || (ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] < 16)){
				  ECU_Motor_MoveForward(&moving_motor);
			  }
			  /* Delay to let the car move Forward to half the car length */
			  HAL_Delay(250);
			  ECU_Motor_Stop(&moving_motor);
			  while(ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] > 6){
				  ECU_Motor_MoveReverseRight(&moving_motor);
			  }
			  ECU_Motor_Stop(&moving_motor);
			  while(ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 4){
				  ECU_Motor_MoveReverseLeft(&moving_motor);
			  }

			  osEventFlagsClear(Car_modeHandle, CAR_CONTROL_BIT);


		  }
		  /* Select Left Side to Park */
		  else if((osEventFlagsGet(Car_modeHandle) & PARKING_SIDE_BIT) || (STD_ACTIVE << PARKING_SIDE_BIT)){
			  printf("Park Left Side\n");
			  while((ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX] < 16) && (ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX] < 16)){
				  ECU_Motor_MoveForward(&moving_motor);
			  }
			  /* Delay to let the car move Forward to half the car length */
			  HAL_Delay(50);
			  ECU_Motor_Stop(&moving_motor);
			  while(ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 3){
				  ECU_Motor_MoveReverseLeft(&moving_motor);
			  }
			  ECU_Motor_Stop(&moving_motor);
			  while(ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX] > 3){
				  ECU_Motor_MoveReverseRight(&moving_motor);
			  }

			  ECU_Motor_ChangeSpeed(&moving_motor, &medium_speed);
			  osEventFlagsClear(Car_modeHandle, CAR_CONTROL_BIT);


		  }
		  else{/* DO NOTHING */}


	  }
	  else{/* DO NOTHING */}
    osDelay(200);
  }
  /* USER CODE END RTOS_Automatic_Parking */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

