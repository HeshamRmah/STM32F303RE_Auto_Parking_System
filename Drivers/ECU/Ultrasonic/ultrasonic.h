/*
 * ultrasonic.h
 *
 *  Created on: Sep 14, 2023
 *      Author: Hesham
 */

#ifndef ECU_ULTRASONIC_ULTRASONIC_H_
#define ECU_ULTRASONIC_ULTRASONIC_H_

/* ----------------- Includes -----------------*/
#include "../ECU_std_types.h"
#include "../../Drivers/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"

/* ----------------- Macro Declarations -----------------*/
#define ULTRASONIC_SENSOR_NUMBER 6
#define ULTRASONIC_DISTANCE_DIVISION_FACTOR ((float)58)

/* ----------------- Macro Functions Declarations -----------------*/

/* ----------------- Data Type Declarations -----------------*/

/* Global Array to save each ultrasonic distance value */
extern volatile double ultrasonic_Distance_Values[ULTRASONIC_SENSOR_NUMBER];

/* Global ultrasonic index to select which sensor to update its value */
extern volatile uint8_t sensor_index;

typedef struct{
	TIM_HandleTypeDef *htim;          // Timer Handler   @ref TIM_HandleTypeDef
	uint32_t PWM_Channel;             // PWM Channel     @ref TIM_Channel TIM Channel
	uint32_t IC_Channel;              // IC Channel      @ref TIM_Channel TIM Channel
	//uint8_t sensor_number;			  // Sensor Number
}ultrasonic_obj_t;

/* ----------------- Software Interfaces Declarations -----------------*/

/**
  * @brief  Generate Trigger to Ultrasonic and Calculate the Distance in Asynchronous way.
  * @param  ultrasonic_obj Ultrasonic handle
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Ultrasonic_Read(ultrasonic_obj_t *ultrasonic_obj);

#endif /* ECU_ULTRASONIC_ULTRASONIC_H_ */
