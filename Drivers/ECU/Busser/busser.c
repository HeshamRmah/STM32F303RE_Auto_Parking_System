/*
 * busser.c
 *
 *  Created on: Oct 2, 2023
 *      Author: Hesham
 */


#include "busser.h"



/**
  * @brief  start the PWM signal generation for the busser.
  *
  * @param  busser_obj: busser handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Busser_GeneratePWM(busser_obj_t *busser_obj){

	uint32_t Period = 0;

	/* Check NULL Pointer */
	if (NULL == busser_obj)
	{
		return ECU_ERROR;
	}

	/* Check the TIM channel state */
	if (TIM_CHANNEL_STATE_GET(busser_obj->htim, busser_obj->PWM_Channel) != HAL_TIM_CHANNEL_STATE_READY)
	{
		return ECU_ERROR;
	}

	/* Convert Frequency (Hz) to Period (us) */
	Period = (uint32_t)(1000000 /(busser_obj->Frequency));

	/* Update The Timer with the new Period */
	busser_obj->htim->Instance->ARR = Period;

	/* Update The Timer with the new Duty cycle */
	busser_obj->htim->Instance->CCR1 = (uint32_t)(Period * (busser_obj->Duty_Cycle));

	/* Starts the PWM signal generation */
	HAL_TIM_PWM_Start(busser_obj->htim, busser_obj->PWM_Channel);

	return ECU_OK;
}


/**
  * @brief  stop the PWM signal generation for the busser.
  *
  * @param  busser_obj: busser handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Busser_StopPWM(busser_obj_t *busser_obj){

	/* Check NULL Pointer */
	if (NULL == busser_obj)
	{
		return ECU_ERROR;
	}

	HAL_TIM_PWM_Stop(busser_obj->htim, busser_obj->PWM_Channel);

	return ECU_OK;
}

