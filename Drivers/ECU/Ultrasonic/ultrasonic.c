/*
 * ultrasonic.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Hesham
 */

#include "ultrasonic.h"

/* Global Array to save each ultrasonic distance value */
volatile double ultrasonic_Distance_Values[ULTRASONIC_SENSOR_NUMBER];

/* Global ultrasonic index to select which sensor to update its value */
volatile uint8_t sensor_index = 0;

/* Helper Function to Generate the Trigger Pulse  */
static ECU_StatusTypeDef ECU_Ultrasonic_GenerateTrigger(ultrasonic_obj_t *ultrasonic_obj, uint32_t Period, float Duty);

/* Helper Function to Capture the Time of the Rising Edge */
static void IC_first_Capture(TIM_HandleTypeDef *htim, uint32_t *IC_Val1, uint8_t *Is_first_Capture);

/* Helper Function to Capture the Time of the Falling Edge and calculate the distance */
static void IC_second_Capture(TIM_HandleTypeDef *htim, uint32_t *IC_Val1, uint32_t *IC_Val2, uint32_t *IC_difference, uint8_t *Is_first_Capture);

/* Implementation of Timer Input Capture Callback function */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	static uint32_t IC_Val1[ULTRASONIC_SENSOR_NUMBER];
	static uint32_t IC_Val2[ULTRASONIC_SENSOR_NUMBER];
	static uint32_t IC_difference[ULTRASONIC_SENSOR_NUMBER];
	static uint8_t Is_first_Capture[ULTRASONIC_SENSOR_NUMBER];

	if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)){
		//printf("---------------> ultrasonic index is %d\n",sensor_index);
		if(Is_first_Capture[sensor_index] == 0){

			/* Stop the generation of PWM Trigger Pulse */
			HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);

			/* Capture the Time of the Rising Edge */
			IC_first_Capture(htim, &IC_Val1[sensor_index], &Is_first_Capture[sensor_index]);

		}
		else if (Is_first_Capture[sensor_index] == 1){

			/* Capture the Time of the Falling Edge and calculate the distance */
			IC_second_Capture(htim, &IC_Val1[sensor_index], &IC_Val2[sensor_index], &IC_difference[sensor_index], &Is_first_Capture[sensor_index]);

			/* Stop the Input Capture to the ECHO */
			HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);

		}

	}
	else{/* Do Nothing */}

}

/**
  * @brief  Generate Trigger to Ultrasonic and Calculate the Distance Asynchronous.
  * @param  ultrasonic_obj Ultrasonic handle
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Ultrasonic_Read(ultrasonic_obj_t *ultrasonic_obj){
	/* Check ERROR NULL Pointer */
	if(NULL == ultrasonic_obj){
		return ECU_ERROR;
	}

	/* Generate 10 us Pulse Trigger to Ultrasonic */
	ECU_Ultrasonic_GenerateTrigger(ultrasonic_obj, 10000, 0.001);

	/* Start the Input Capture to Capture the ultrasonic ECOH  */
	HAL_TIM_IC_Start_IT(ultrasonic_obj->htim, ultrasonic_obj->IC_Channel);

	return ECU_OK;
}

static ECU_StatusTypeDef ECU_Ultrasonic_GenerateTrigger(ultrasonic_obj_t *ultrasonic_obj, uint32_t Period, float Duty){

	/* Check NULL Pointer */
	if (NULL == ultrasonic_obj)
	{
		return ECU_ERROR;
	}
	/* Check the TIM channel state */
	if (TIM_CHANNEL_STATE_GET(ultrasonic_obj->htim, ultrasonic_obj->PWM_Channel) != HAL_TIM_CHANNEL_STATE_READY)
	{
		return ECU_ERROR;
	}

	/* Update configuration with desired period */
	ultrasonic_obj->htim->Instance->ARR = Period;

	/* Update configuration with desired Duty cycle */
	ultrasonic_obj->htim->Instance->CCR1 = (Period * Duty);

	/* Starts the PWM Trigger signal generation */
	HAL_TIM_PWM_Start(ultrasonic_obj->htim, ultrasonic_obj->PWM_Channel);

	return ECU_OK;;
}

static void IC_first_Capture(TIM_HandleTypeDef *htim, uint32_t *IC_Val1, uint8_t *Is_first_Capture){

	/* Get the time of Rising Edge */
	*IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

	/* Update the period so that no overflow (Due to setting the trigger period) happens */
	htim->Instance->ARR = 0xffff;

	/* Update Is_first_Capture value */
	*Is_first_Capture = 1;

	/* Setting the Timer Sensing Edge to Falling Edge */
	__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
}

static void IC_second_Capture(TIM_HandleTypeDef *htim, uint32_t *IC_Val1, uint32_t *IC_Val2, uint32_t *IC_difference, uint8_t *Is_first_Capture){

	/* Get the time of Falling Edge */
	*IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

	/* Setting the Timer Counter Register to Zero */
	__HAL_TIM_SET_COUNTER(htim, 0);

	/* Checking the values */
	if ((*IC_Val2) >= (*IC_Val1)){
		*IC_difference = (*IC_Val2) - (*IC_Val1);
	}
	else{
		// only if there is an overflow (very unlikely scenario)
		*IC_difference = (0xffff - *IC_Val1) + (*IC_Val2);
		//printf("sensor %d --------> overflow\n",sensor_index+1);
	}

	/* Update the desired ultrasonic distance value */
	ultrasonic_Distance_Values[sensor_index] = (*IC_difference) / ULTRASONIC_DISTANCE_DIVISION_FACTOR;

	/* Update Is_first_Capture value */
	*Is_first_Capture = 0;

	/* Setting the Timer Sensing Edge to Rasing Edge */
	__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
}
