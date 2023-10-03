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
#include "../../Drivers/ECU/IR/ecu_ir.h"
#include "../../Drivers/ECU/Busser/busser.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAR_CONTROL_BIT          (1 << 0)
#define PARKING_SIDE_BIT         (1 << 1)
#define RIGHT_PARKING_SIDE       (0 << 1)
#define LEFT_PARKING_SIDE        (1 << 1)
#define MANUAL_CONTROL_MODE      (0)
#define AUTOMATIC_PARKING_MODE   (1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

ultrasonic_obj_t Front_ultrasonic      = {.htim = &htim1, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t RightFront_ultrasonic = {.htim = &htim2, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t RightRear_ultrasonic  = {.htim = &htim3, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t LeftFront_ultrasonic  = {.htim = &htim4, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t LeftRear_ultrasonic   = {.htim = &htim8, .PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

ultrasonic_obj_t Rear_ultrasonic       = {.htim = &htim15,.PWM_Channel = TIM_CHANNEL_1, .IC_Channel = TIM_CHANNEL_2};

busser_obj_t busser       = {.htim = &htim17, .PWM_Channel = TIM_CHANNEL_1, .Frequency = 2500, .Duty_Cycle = 0.25};

motor_obj_t moving_motor  = {.htim = &htim16, .PWM_Channel = TIM_CHANNEL_1, .speed.Frequency = 25000, .speed.Duty_Cycle = 0.9};

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
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for car_next_step */
osThreadId_t car_next_stepHandle;
const osThreadAttr_t car_next_step_attributes = {
  .name = "car_next_step",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for Automatic_Parking */
osThreadId_t Automatic_ParkingHandle;
const osThreadAttr_t Automatic_Parking_attributes = {
  .name = "Automatic_Parking",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
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


  /* Infinite loop */
  for(;;)
  {
	  //printf("DefaultTask is Running\n");
	  osThreadSuspend(Automatic_ParkingHandle);
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
	  /*
	  if(ECU_IR_IsActive(&IR_Right_Rear) == IR_Active_STATE){
		  ECU_Busser_GeneratePWM(&busser);
	  }
	  else{
		  ECU_Busser_StopPWM(&busser);
	  }*/
	  printf("IR_Right_Rear in = %d\n",ECU_IR_IsActive(&IR_Right_Rear));
	  printf("IR_Left_Rear in = %d\n",ECU_IR_IsActive(&IR_Left_Rear));

	  if((osEventFlagsGet(Car_modeHandle) & PARKING_SIDE_BIT) == RIGHT_PARKING_SIDE)
	  {
		  ECU_Ultrasonic_Read(&RightRear_ultrasonic);
		  ECU_Ultrasonic_Read(&Front_ultrasonic);
		  ECU_Ultrasonic_Read(&Rear_ultrasonic);
		  ECU_Ultrasonic_Read(&RightFront_ultrasonic);
	  }
	  else if((osEventFlagsGet(Car_modeHandle) & PARKING_SIDE_BIT) == LEFT_PARKING_SIDE)
	  {
		  ECU_Ultrasonic_Read(&LeftRear_ultrasonic);
		  ECU_Ultrasonic_Read(&Front_ultrasonic);
		  ECU_Ultrasonic_Read(&Rear_ultrasonic);
		  ECU_Ultrasonic_Read(&LeftFront_ultrasonic);
	  }
	  else{}

	  //ECU_Ultrasonic_Read(&LeftFront_ultrasonic);
	  //ECU_Ultrasonic_Read(&LeftRear_ultrasonic);

	  printf("Front_ultrasonic distance is %f\n",ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX]);
	  printf("RightFront_ultrasonic distance is %f\n",ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX]);
	  printf("RightRear_ultrasonic distance is %f\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);
	  printf("LeftFront_ultrasonic distance is %f\n",ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX]);
	  printf("LeftRear_ultrasonic distance is %f\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);
	  printf("Rear_ultrasonic distance is %f\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);

    osDelay(70);
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
		  osEventFlagsSet(Car_modeHandle, CAR_CONTROL_BIT);
		  osEventFlagsClear(Car_modeHandle, PARKING_SIDE_BIT);
		  osThreadResume(Automatic_ParkingHandle);
		  osThreadSuspend(car_next_stepHandle);
	  }
	  else if( (Bluetooth_RX_Data == AUTO_PARK_LEFT) ){
		  osEventFlagsSet(Car_modeHandle, CAR_CONTROL_BIT);
		  osEventFlagsSet(Car_modeHandle, PARKING_SIDE_BIT);
		  osThreadResume(Automatic_ParkingHandle);
		  osThreadSuspend(car_next_stepHandle);
	  }
	  else{/* DO NOTHING */}

	  if( (osEventFlagsGet(Car_modeHandle) & CAR_CONTROL_BIT) == MANUAL_CONTROL_MODE){
		  //printf("Mobile Control Mode\n");
		  ECU_Motor_NextStep(&moving_motor, &Bluetooth_RX_Data);
	  }
	  else{/* DO NOTHING */}

	  osDelay(150);
  }
  /* USER CODE END RTOS_Car_Next_Step */
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
	uint8_t front_ultrasonic = 0;
	uint8_t rear_ultrasonic = 0;
	uint32_t time_out = 0;
  /* Infinite loop */
  for(;;)
  {
	  if( (Bluetooth_RX_Data == AUTO_PARK_OFF) ){
		  osEventFlagsClear(Car_modeHandle, CAR_CONTROL_BIT);
	  }
	  else{/* DO NOTHING */}

	  if( (osEventFlagsGet(Car_modeHandle) & CAR_CONTROL_BIT) == AUTOMATIC_PARKING_MODE)
	  {
		  printf("-------------------------- Automatic Parking Mode ----------------------\n");

		  /* changing the speed to parking speed */
		  ECU_Motor_ChangeSpeed(&moving_motor, &medium_speed);

		  /* Selected Right Side to Park */
		  if((osEventFlagsGet(Car_modeHandle) & PARKING_SIDE_BIT) == RIGHT_PARKING_SIDE)
		  {
			  printf("Parking Right Side\n");

			  /* storing the ultra-sonic initial distance before parking */
			  front_ultrasonic = ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX];
			  rear_ultrasonic = ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX];

			  /* MoveForward so it can Searching for Empty Slot */
			  ECU_Motor_MoveForward();
			  while((ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX] < (front_ultrasonic + 15)) || (ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] < (rear_ultrasonic + 15)))
			  {
				  //printf("RightFront_ultrasonic distance is %f\n",ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX]);
				  //printf("RightRear_ultrasonic distance is %f\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);
				  printf("Searching for Empty Slot\n");
			  }
			  printf("Found the Empty Slot\n");

			  /* Delay to let the car move Forward to about car length */
			  while((time_out != 0xff0) && ((ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX] > (front_ultrasonic + 17)) || (ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] > (rear_ultrasonic + 15))))
			  {
				  printf("Centering the car after Finding the Empty Slot\n");
				  time_out++;
			  }

			  /* Breaking Sequence */
			  ECU_Motor_MoveReverse();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* reduce the speed while Reversing to deal with motor hardware inefficiency */
			  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);

			  ECU_Motor_MoveReverse();
			  HAL_Delay(1400);
			  ECU_Motor_Stop();

			  /* MoveReverseRight so the car rear can enter the Empty Slot */
			  ECU_Motor_MoveReverseRight();
			  while((ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 30))
			  {
				  //printf("IR_Right_Rear in = %d",ECU_IR_IsActive(&IR_Right_Rear));
				  //printf("RightRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);
				  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);
				  printf("trying to Park 1 Reverse Right\n");
			  }
			  //printf("RightRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);
			  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);

			  /* Breaking Sequence */
			  ECU_Motor_MoveForward();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* ReverseLeft to Adjust the position better in the slot */
			  ECU_Motor_MoveReverseLeft();
			  while((ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] > 4) && (ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 4))
			  {
				  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);
				  //printf("RightRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);
				  printf("trying to Park 2 Reverse Left\n");
			  }
			  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);
			  //printf("RightRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX]);

			  /* Breaking Sequence */
			  ECU_Motor_MoveForward();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* return the speed back to parking speed */
			  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);

			  /* MoveForwardRight to Adjust the position of the front of the car */
			  ECU_Motor_MoveForwardRight();
			  while((ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX] > 5) && (ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX] > 13))
			  {
				  printf("trying to Park 3 Forward Right\n");
			  }

			  /* Breaking Sequence */
			  ECU_Motor_MoveReverse();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* reduce the speed while Reversing to deal with motor hardware inefficiency */
			  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);

			  /* ReverseLeft to Adjust the position better in the slot */
			  ECU_Motor_MoveReverseLeft();
			  HAL_Delay(300);
			  ECU_Motor_Stop();

			  /* MoveReverse to Adjust the position better in the slot */
			  ECU_Motor_MoveReverse();
			  while( (ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 20) )
			  {
				  printf("try to Park 4 Reverse \n");
			  }

			  /* Breaking Sequence */
			  ECU_Motor_MoveForward();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  ECU_Motor_MoveForwardRight(&moving_motor);
			  while((ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX] > 10 ) && (ultrasonic_Distance_Values[RIGHT_FRONT_ULTRASONIC_INDEX] > 12) /*&& (ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] > 4)*/)
			  {
				  printf("try to Park 5 Forward Right\n");
			  }
			  ECU_Motor_MoveReverse();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

		  }
		  /* ----------------------------------------------- Selected Left Side to Park -------------------------------------------- */
		  else if((osEventFlagsGet(Car_modeHandle) & PARKING_SIDE_BIT) == LEFT_PARKING_SIDE)
		  {
			  printf("Park Left Side\n");

			  /* storing the ultra-sonic initial distance before parking */
			  front_ultrasonic = ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX];
			  rear_ultrasonic = ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX];

			  /* MoveForward so it can Searching for Empty Slot */
			  ECU_Motor_MoveForward();
			  while((ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX] < (front_ultrasonic + 15)) || (ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX] < (rear_ultrasonic + 15)))
			  {
				  //printf("LeftFront_ultrasonic distance is %f\n",ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX]);
				  //printf("LeftRear_ultrasonic distance is %f\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);
				  printf("Searching for Empty Slot\n");
			  }
			  printf("Found the Empty Slot\n");

			  /* Delay to let the car move Forward to about car length */
			  while((time_out != 0xff0) && ((ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX] > (front_ultrasonic + 17)) || (ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX] > (rear_ultrasonic + 15))))
			  {
				  printf("Centering the car after Finding the Empty Slot\n");
				  time_out++;
			  }

			  /* Breaking Sequence */
			  ECU_Motor_MoveReverse();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* reduce the speed while Reversing to deal with motor hardware inefficiency */
			  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);

			  ECU_Motor_MoveReverse();
			  HAL_Delay(1400);
			  ECU_Motor_Stop();

			  /* MoveReverseLeft so the car rear can enter the Empty Slot */
			  ECU_Motor_MoveReverseLeft();
			  while((ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 20))
			  {
				  //printf("IR_Left_Rear in = %d",ECU_IR_IsActive(&IR_Left_Rear));
				  //printf("LeftRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);
				  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);
				  printf("trying to Park 1 Reverse Left\n");
			  }
			  //printf("LeftRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);
			  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);

			  /* Breaking Sequence */
			  ECU_Motor_MoveForward();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* ReverseRight to Adjust the position better in the slot */
			  ECU_Motor_MoveReverseRight();
			  while((ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX] > 4) && (ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 4))
			  {
				  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);
				  //printf("LeftRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);
				  printf("trying to Park 2 Reverse Right\n");
			  }
			  //printf("Rear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX]);
			  //printf("LeftRear_ultrasonic distance is %lu\n",ultrasonic_Distance_Values[LEFT_REAR_ULTRASONIC_INDEX]);

			  /* Breaking Sequence */
			  ECU_Motor_MoveForward();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* return the speed back to parking speed */
			  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);

			  /* MoveForwardLeft to Adjust the position of the front of the car */
			  ECU_Motor_MoveForwardLeft();
			  while((ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX] > 5) && (ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX] > 13))
			  {
				  printf("trying to Park 3 Forward Left\n");
			  }

			  /* Breaking Sequence */
			  ECU_Motor_MoveReverse();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  /* reduce the speed while Reversing to deal with motor hardware inefficiency */
			  ECU_Motor_ChangeSpeed(&moving_motor, &low_speed);

			  /* ReverseRight to Adjust the position better in the slot */
			  ECU_Motor_MoveReverseRight();
			  HAL_Delay(300);
			  ECU_Motor_Stop();

			  /* MoveReverse to Adjust the position better in the slot */
			  ECU_Motor_MoveReverse();
			  while( (ultrasonic_Distance_Values[REAR_ULTRASONIC_INDEX] > 20) )
			  {
				  printf("try to Park 4 Reverse \n");
			  }

			  /* Breaking Sequence */
			  ECU_Motor_MoveForward();
			  HAL_Delay(5);
			  ECU_Motor_Stop();

			  ECU_Motor_MoveForwardLeft(&moving_motor);
			  while((ultrasonic_Distance_Values[FRONT_ULTRASONIC_INDEX] > 10 ) && (ultrasonic_Distance_Values[LEFT_FRONT_ULTRASONIC_INDEX] > 9) /*&& (ultrasonic_Distance_Values[RIGHT_REAR_ULTRASONIC_INDEX] > 4)*/)
			  {
				  printf("try to Park 5 Forward Left\n");
			  }
			  ECU_Motor_MoveReverse();
			  HAL_Delay(5);
			  ECU_Motor_Stop();
		  }
		  else{/* DO NOTHING */}

		  /* terminating the Auto Parking mode */
		  printf("Finally ... the car should be parked successfully by now\n");
		  osEventFlagsClear(Car_modeHandle, CAR_CONTROL_BIT);
		  ECU_Motor_ChangeSpeed(&moving_motor, &high_speed);
		  //Bluetooth_RX_Data = 'S';
		  osThreadResume(car_next_stepHandle);
		  osThreadSuspend(Automatic_ParkingHandle);
	  }
	  else{/* DO NOTHING */}
    osDelay(120000);
  }
  /* USER CODE END RTOS_Automatic_Parking */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

