/*
 * bluetooth.c
 *
 *  Created on: Sep 17, 2023
 *      Author: Hesham
 */

#include "bluetooth.h"

uint8_t Bluetooth_RX_Data = 'S';
uint8_t temp = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//printf("----------------------> is %c\n",temp);
	if( BLUETOOTH_CONDITION(temp) ){
		Bluetooth_RX_Data = temp;
		//printf("ISR is %c\n",Bluetooth_RX_Data);
	}
	HAL_UART_Receive_IT(huart, &temp, 1);
}


/**
  * @brief  .
  *
  * @param  bluetooth_obj: bluetooth handle
  *
  * @retval ECU status
  */
ECU_StatusTypeDef ECU_Bluetooth_ReciveData(const bluetooth_obj_t *bluetooth_obj){

	/* Check NULL Pointer */
	if (NULL == bluetooth_obj)
	{
		return ECU_ERROR;
	}

	HAL_UART_Receive_IT(bluetooth_obj->huart, &temp, bluetooth_obj->Numberofdata);

	return ECU_OK;
}
