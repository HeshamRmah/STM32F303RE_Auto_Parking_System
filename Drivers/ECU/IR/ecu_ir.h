/*
 * ecu_ir.h
 *
 *  Created on: Oct 2, 2023
 *      Author: Hesham
 */

#ifndef ECU_IR_ECU_IR_H_
#define ECU_IR_ECU_IR_H_

/* ----------------- Includes -----------------*/
#include "../../Drivers/STM32F3xx_HAL_Driver/Inc/stm32f3xx_hal.h"
#include "../ECU_std_types.h"

/* ----------------- Macro Declarations -----------------*/

#define IR_IDLE_STATE          (1)
#define IR_Active_STATE        (0)


/* ----------------- Macro Functions Declarations -----------------*/


/* ----------------- Data Type Declarations -----------------*/
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
}ir_obj_t;

extern ir_obj_t IR_Right_Rear;
extern ir_obj_t IR_Left_Rear;

/* ----------------- Software Interfaces Declarations -----------------*/
/**
  * @brief  Get the status of the IR.
  *
  * @param  ir_obj_t: IR handle
  *
  * @retval status of the IR
  */
uint8_t ECU_IR_IsActive(ir_obj_t *ir_obj);



#endif /* ECU_IR_ECU_IR_H_ */
