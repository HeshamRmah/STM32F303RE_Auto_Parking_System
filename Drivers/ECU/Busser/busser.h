/*
 * busser.h
 *
 *  Created on: Oct 2, 2023
 *      Author: Hesham
 */

#ifndef ECU_BUSSER_BUSSER_H_
#define ECU_BUSSER_BUSSER_H_

/* ----------------- Includes -----------------*/
#include "../ECU_std_types.h"
#include "../../Drivers/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"

/* ----------------- Macro Declarations -----------------*/


/* ----------------- Macro Functions Declarations -----------------*/


/* ----------------- Data Type Declarations -----------------*/

typedef struct{
	TIM_HandleTypeDef *htim;          // Timer Handler      @ref TIM_HandleTypeDef
	uint32_t PWM_Channel;               // PWM Channel      @ref TIM_Channel TIM Channel
	uint32_t Frequency;
	float Duty_Cycle;
}busser_obj_t;


/* ----------------- Software Interfaces Declarations -----------------*/

/**
  * @brief  start the PWM signal generation for the busser.
  *
  * @param  busser_obj: busser handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Busser_GeneratePWM(busser_obj_t *busser_obj);

/**
  * @brief  stop the PWM signal generation for the busser.
  *
  * @param  busser_obj: busser handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Busser_StopPWM(busser_obj_t *busser_obj);

#endif /* ECU_BUSSER_BUSSER_H_ */
