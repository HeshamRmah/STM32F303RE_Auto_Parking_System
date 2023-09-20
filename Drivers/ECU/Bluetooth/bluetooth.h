/*
 * bluetooth.h
 *
 *  Created on: Sep 17, 2023
 *      Author: Hesham
 */

#ifndef BLUETOOTH_BLUETOOTH_H_
#define BLUETOOTH_BLUETOOTH_H_

/* ----------------- Includes -----------------*/
#include "../../Drivers/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
#include "../ECU_std_types.h"

/* ----------------- Macro Declarations -----------------*/
#define FRONT_ULTRASONIC_INDEX         0
#define RIGHT_FRONT_ULTRASONIC_INDEX   1
#define RIGHT_REAR_ULTRASONIC_INDEX    2
#define LEFT_FRONT_ULTRASONIC_INDEX    3
#define LEFT_REAR_ULTRASONIC_INDEX     4
#define REAR_ULTRASONIC_INDEX          5

/* ----------------- Macro Functions Declarations -----------------*/


/* ----------------- Data Type Declarations -----------------*/
extern uint8_t Bluetooth_RX_Data;

typedef struct{
	UART_HandleTypeDef *huart;          // UART Handler   @ref UART_HandleTypeDef
	uint8_t Numberofdata;
}bluetooth_obj_t;

/* ----------------- Software Interfaces Declarations -----------------*/

/**
  * @brief  .
  *
  * @param  bluetooth_obj: bluetooth handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Bluetooth_ReciveData(const bluetooth_obj_t *bluetooth_obj);

#endif /* BLUETOOTH_BLUETOOTH_H_ */
