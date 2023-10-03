/*
 * ecu_motor.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Hesham
 */

#ifndef INC_ECU_MOTOR_H_
#define INC_ECU_MOTOR_H_

/* ----------------- Includes -----------------*/
#include "../../Drivers/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
#include "../ECU_std_types.h"

/* ----------------- Macro Declarations -----------------*/
#define STEERING_MOTOR_PORT                 GPIOC
#define STEERING_MOTOR_POSITIVE_PIN         GPIO_PIN_8
#define STEERING_MOTOR_NEGITVE_PIN          GPIO_PIN_9

//#define STEERING_MOTOR_EN_PORT              GPIOA
//#define STEERING_MOTOR_EN_PIN               GPIO_PIN_8

#define MOVING_MOTOR_PORT                   GPIOA
#define MOVING_MOTOR_POSITIVE_PIN           GPIO_PIN_9
#define MOVING_MOTOR_NEGITVE_PIN            GPIO_PIN_10


#define FORWARD         'F'
#define REVERSE         'B'
#define FORWARD_LEFT    'G'
#define FORWARD_RIGHT   'I'
#define REVERSE_LEFT    'H'
#define REVERSE_RIGHT   'J'
#define LEFT            'L'
#define RIGHT           'R'
#define STOP            'S'
#define AUTO_PARK_RIGHT 'W'
#define AUTO_PARK_LEFT  'U'
#define AUTO_PARK_OFF   'w'
#define HIGH_SPEED      '#'
#define MEDIUM_SPEED    '@'
#define LOW_SPEED       '!'


/* ----------------- Macro Functions Declarations -----------------*/

/* ----------------- Data Type Declarations -----------------*/
typedef struct{
	uint32_t Frequency;             // Frequency of the PWM Signal
	float Duty_Cycle;               // Duty Cycle of the PWM Signal values from 0 to 1
}motor_speed_t;

typedef struct{
	TIM_HandleTypeDef *htim;          // Timer Handler      @ref TIM_HandleTypeDef
	uint32_t PWM_Channel;             // PWM Channel        @ref TIM_Channel TIM Channel
	motor_speed_t speed;              // Speed of the motor @ref motor_speed_t
}motor_obj_t;

extern motor_speed_t high_speed;

extern motor_speed_t medium_speed;

extern motor_speed_t low_speed;

/* ----------------- Software Interfaces Declarations -----------------*/

/**
  * @brief  Starts the PWM signal generation.
  */
ECU_StatusTypeDef ECU_Motor_GeneratePWM(motor_obj_t *motor_obj);

/**
  * @brief  Stop the PWM signal generation.
  */
ECU_StatusTypeDef ECU_Motor_StopPWM(motor_obj_t *motor_obj);

/**
  * @brief  Change the Speed of the car.
  */
ECU_StatusTypeDef ECU_Motor_ChangeSpeed(motor_obj_t *motor_obj, const motor_speed_t *speed);

/**
  * @brief  Stop the PWM signal generation.
  */
ECU_StatusTypeDef ECU_Motor_NextStep(motor_obj_t *motor_obj, uint8_t *direction);

/**
  * @brief  Move the Car Forward.
  */
ECU_StatusTypeDef ECU_Motor_MoveForward();

/**
  * @brief  Move the Car Reverse.
  */
ECU_StatusTypeDef ECU_Motor_MoveReverse();

/**
  * @brief  Move the Car Forward Right.
  */
ECU_StatusTypeDef ECU_Motor_MoveForwardRight();

/**
  * @brief  Move the Car Forward Left.
  */
ECU_StatusTypeDef ECU_Motor_MoveForwardLeft();

/**
  * @brief  Move the Car Reverse Right.
  */
ECU_StatusTypeDef ECU_Motor_MoveReverseRight();

/**
  * @brief  Move the Car Reverse Left.
  */
ECU_StatusTypeDef ECU_Motor_MoveReverseLeft();

/**
  * @brief  Stop the Car.
  */
ECU_StatusTypeDef ECU_Motor_Stop();

/**
  * @brief  Move the Steering Motor Right.
  */
ECU_StatusTypeDef ECU_Motor_MoveRight();

/**
  * @brief  Move the Steering Motor Left.
  */
ECU_StatusTypeDef ECU_Motor_MoveLeft();

#endif /* INC_ECU_MOTOR_H_ */
