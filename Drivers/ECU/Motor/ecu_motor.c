/*
 * ecu_motor.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Hesham
 */


#include "ecu_motor.h"


/**
  * @brief  Starts the PWM signal generation.
  * @param  motor_obj Motor handle
  * @param  Period of the PWM signal in Microseconds
  * @param  Duty Cycle for the generated wave
  *          This parameter can be in the floating range values of 0 to 1:
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_GeneratePWM(motor_obj_t *motor_obj){

	uint32_t Period = 0;

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	/* Check the TIM channel state */
	if (TIM_CHANNEL_STATE_GET(motor_obj->htim, motor_obj->PWM_Channel) != HAL_TIM_CHANNEL_STATE_READY)
	{
		return ECU_ERROR;
	}

	/*	TIM_OC_InitTypeDef
		OCPolarity = TIM_OCPOLARITY_LOW
		OCNPolarity = TIM_OCNPOLARITY_HIGH
		OCFastMode = TIM_OCFAST_DISABLE
		OCIdleState = TIM_OCIDLESTATE_RESET
		OCNIdleState = TIM_OCNIDLESTATE_RESET
	*/

	Period = (uint32_t)(1000000 /(motor_obj->speed.Frequency));

	/* Update The Timer with the new Period */
	motor_obj->htim->Instance->ARR = Period;

	/* Update The Timer with the new Duty cycle */
	motor_obj->htim->Instance->CCR1 = (uint32_t)(Period * (motor_obj->speed.Duty_Cycle));

	/* Starts the PWM signal generation */
	HAL_TIM_PWM_Start(motor_obj->htim, motor_obj->PWM_Channel);

	return ECU_OK;
}

/**
  * @brief  Stop the PWM signal generation.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_StopPWM(motor_obj_t *motor_obj){
	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	/* Stop the PWM signal generation */
	HAL_TIM_PWM_Stop(motor_obj->htim, motor_obj->PWM_Channel);

	return ECU_OK;
}

/**
  * @brief  Stop the PWM signal generation.
  *
  * @param  motor_obj: Motor handle
  * @param  direction: Direction of the Motor
  * @param  speed: Speed of the Motor
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_NextStep(motor_obj_t *motor_obj, uint8_t *direction){
	/* Check NULL Pointer */
	if ((NULL == motor_obj) || (NULL == direction))
	{
		return ECU_ERROR;
	}

	/* Stop the PWM signal generation */
	switch(*direction)
	{
		case FORWARD:       ECU_Motor_MoveForward     (motor_obj); break;
		case REVERSE:       ECU_Motor_MoveReverse     (motor_obj); break;
		case FORWARD_LEFT:  ECU_Motor_MoveForwardLeft (motor_obj); break;
		case FORWARD_RIGHT: ECU_Motor_MoveForwardRight(motor_obj); break;
		case REVERSE_LEFT:  ECU_Motor_MoveReverseLeft (motor_obj); break;
		case REVERSE_RIGHT: ECU_Motor_MoveReverseRight(motor_obj); break;
		case RIGHT:         ECU_Motor_MoveRight       (motor_obj); break;
		case LEFT:          ECU_Motor_MoveLeft        (motor_obj); break;
		case STOP:          ECU_Motor_Stop            (motor_obj); break;
		default :           ECU_Motor_Stop            (motor_obj); break;
	}

	return ECU_OK;
}

/**
  * @brief  Move the Car Forward.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveForward(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("Motor_MoveForward\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Reverse.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveReverse(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveReverse\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Forward Right.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveForwardRight(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveForwardRight\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);
	//HAL_Delay(50);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Forward Left.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveForwardLeft(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveForwardLeft\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);
	//HAL_Delay(50);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}


/**
  * @brief  Move the Car Reverse Right.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveReverseRight(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveReverseRight\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);
	//HAL_Delay(50);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Reverse Left.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveReverseLeft(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveReverseLeft\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);
	//HAL_Delay(50);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	return ECU_OK;
}

/**
  * @brief  Stop the Car.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_Stop(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("Stop\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);
	//HAL_Delay(50);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}

/**
  * @brief  Move the Steering Motor Right.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveRight(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveRight\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);


	return ECU_OK;
}

/**
  * @brief  Move the Steering Motor Left.
  *
  * @param  motor_obj: Motor handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveLeft(motor_obj_t *motor_obj){

	/* Check NULL Pointer */
	if (NULL == motor_obj)
	{
		return ECU_ERROR;
	}

	printf("MoveLeft\n");

	HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);


	return ECU_OK;
}
