/*
 * ecu_ir.c
 *
 *  Created on: Oct 2, 2023
 *      Author: Hesham
 */

#include "ecu_ir.h"


extern ir_obj_t IR_Right_Rear = {.port = GPIOC, .pin = GPIO_PIN_12};
extern ir_obj_t IR_Left_Rear  = {.port = GPIOB, .pin = GPIO_PIN_0};


/**
  * @brief  Get the status of the IR.
  *
  * @param  ir_obj_t: IR handle       @ref ir_obj_t
  *
  * @retval status of the IR          IR_IDLE_STATE OR IR_Active_STATE
  *
  */
uint8_t ECU_IR_IsActive(ir_obj_t *ir_obj){
	return HAL_GPIO_ReadPin(ir_obj->port, ir_obj->pin) ? IR_IDLE_STATE: IR_Active_STATE;
}
