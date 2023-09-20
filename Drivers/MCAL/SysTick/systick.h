/*
 * systick.h
 *
 *  Created on: Aug 14, 2023
 *      Author: Hesham
 */

#ifndef SYSTICK_SYSTICK_H_
#define SYSTICK_SYSTICK_H_

#include "../mcal_std_types.h"
#include "systick_reg.h"




#define SysTick_ENABLE()                                    (SysTick.STK_CTRL.ENABLE = 1)
#define SysTick_DISABLE()                                   (SysTick.STK_CTRL.ENABLE = 0)

#define SysTick_EXCEPTION_REQUEST_ENABLE()                  (SysTick.STK_CTRL.TICKINT = 1)
#define SysTick_EXCEPTION_REQUEST_DISABLE()                 (SysTick.STK_CTRL.TICKINT = 0)

#define SysTick_CLKSCR_AHB()                                (SysTick.STK_CTRL.CLKSOURCE = 1)
#define SysTick_CLKSCR_AHB_DIV_BY_8()                       (SysTick.STK_CTRL.CLKSOURCE = 0)

#define SysTick_GET_FLAG()                                  (SysTick.STK_CTRL.COUNTFLAG)


typedef enum {
	SysTick_POLLING_REQUEST = 0,
	SysTick_EXCEPTION_REQUEST
}systick_exception_mode;

typedef enum {
	SysTick_CLK_SCR_AHB= 0,
	SysTick_CLK_SCR_AHB_DIV_BY_8
}systick_clock_source;



typedef struct{
	systick_exception_mode exception_mode;
	systick_clock_source clock_source;
	uint32 value;
}Systick_obj_t;





Std_ReturnType systick_intialize(const Systick_obj_t *_systick_config);
Std_ReturnType systick_Start(const Systick_obj_t *_systick_config);
Std_ReturnType systick_Stop(const Systick_obj_t *_systick_config);
Std_ReturnType systick_SetTime(Systick_obj_t *_systick_config, uint32 value);




#endif /* SYSTICK_SYSTICK_H_ */
