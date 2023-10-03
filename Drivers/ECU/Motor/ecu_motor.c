/*
 * ecu_motor.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Hesham
 */


#include "ecu_motor.h"

motor_speed_t high_speed   = {.Frequency = 25000, .Duty_Cycle = 0.9};

motor_speed_t medium_speed = {.Frequency = 25000, .Duty_Cycle = 0.845};

motor_speed_t low_speed    = {.Frequency = 25000, .Duty_Cycle = 0.77};


/**
  * @brief  Starts the PWM signal generation.
  *
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

	/* Convert Frequency (Hz) to Period (us) */
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
  * @brief  Change the Speed of the car.
  *
  * @param  motor_obj: Motor handle
  * @param  speed: Speed of motor
  *         the can be either ( &high_speed or &medium_speed or &low_speed )
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_ChangeSpeed(motor_obj_t *motor_obj, const motor_speed_t *speed){
	/* Check NULL Pointer */
	if ( (NULL == motor_obj) || (NULL == speed))
	{
		return ECU_ERROR;
	}

	/* Stop the old PWM */
	ECU_Motor_StopPWM(motor_obj);

	/* Update the Frequency of PWM Signal */
	motor_obj->speed.Frequency = speed->Frequency;

	/* Update the Duty Cycle of PWM Signal */
	motor_obj->speed.Duty_Cycle = speed->Duty_Cycle;

	/* Starts the Updated PWM signal generation */
	ECU_Motor_GeneratePWM(motor_obj);

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
		case FORWARD:       ECU_Motor_MoveForward     (); break;
		case REVERSE:       ECU_Motor_MoveReverse     (); break;
		case FORWARD_LEFT:  ECU_Motor_MoveForwardLeft (); break;
		case FORWARD_RIGHT: ECU_Motor_MoveForwardRight(); break;
		case REVERSE_LEFT:  ECU_Motor_MoveReverseLeft (); break;
		case REVERSE_RIGHT: ECU_Motor_MoveReverseRight(); break;
		case RIGHT:         ECU_Motor_MoveRight       (); break;
		case LEFT:          ECU_Motor_MoveLeft        (); break;
		case STOP:          ECU_Motor_Stop            (); break;
		case HIGH_SPEED:    ECU_Motor_ChangeSpeed     (motor_obj, &high_speed);
		case MEDIUM_SPEED:  ECU_Motor_ChangeSpeed     (motor_obj, &medium_speed);
		case LOW_SPEED:     ECU_Motor_ChangeSpeed     (motor_obj, &low_speed);
		default:            ECU_Motor_Stop            (); break;
	}

	return ECU_OK;
}

/**
  * @brief  Move the Car Forward.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveForward(){


	//printf("Motor_MoveForward\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Reverse.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveReverse(){


	//printf("MoveReverse\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Forward Right.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveForwardRight(){


	//printf("MoveForwardRight\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Forward Left.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveForwardLeft(){


	//printf("MoveForwardLeft\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}


/**
  * @brief  Move the Car Reverse Right.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveReverseRight(){


	//printf("MoveReverseRight\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	return ECU_OK;
}

/**
  * @brief  Move the Car Reverse Left.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveReverseLeft(){


	//printf("MoveReverseLeft\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	return ECU_OK;
}

/**
  * @brief  Stop the Car.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_Stop(){


	//printf("Stop\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

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

	//printf("MoveRight\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}

/**
  * @brief  Move the Steering Motor Left.
  *
  * @param
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Motor_MoveLeft(){


	//printf("MoveLeft\n");

	//HAL_GPIO_WritePin(STEERING_MOTOR_EN_PORT, STEERING_MOTOR_EN_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEERING_MOTOR_PORT, STEERING_MOTOR_NEGITVE_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_POSITIVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOVING_MOTOR_PORT, MOVING_MOTOR_NEGITVE_PIN, GPIO_PIN_RESET);

	return ECU_OK;
}
