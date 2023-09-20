/*
 * systick_reg.h
 *
 *  Created on: Aug 14, 2023
 *      Author: Hesham
 */

#ifndef SYSTICK_SYSTICK_REG_H_
#define SYSTICK_SYSTICK_REG_H_

#include "systick_reg.h"
#include "../mcal_std_types.h"


#define SysTick_BASE              (0xE000E010)

typedef struct{
	uint32 ENABLE        :1 ;
	uint32 TICKINT       :1 ;
	uint32 CLKSOURCE     :1 ;
	uint32               :13 ;
	uint32 COUNTFLAG     :1 ;
	uint32               :15 ;
}STK_CTRL_t;

typedef struct{
	uint32 value        :24 ;
	uint32              :8 ;
}STK_data_t;

typedef struct{
	STK_CTRL_t STK_CTRL;
	STK_data_t STK_LOAD;
	STK_data_t STK_VAL;
	uint32 STK_CALIB ;
}SysTick_t;


#define SysTick    (*(volatile SysTick_t *)SysTick_BASE)






#endif /* SYSTICK_SYSTICK_REG_H_ */
