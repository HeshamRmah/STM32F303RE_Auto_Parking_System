/*
 * mcal_rcc.h
 *
 *  Created on: Aug 11, 2023
 *      Author: Hesham
 */

#ifndef RCC_MCAL_RCC_H_
#define RCC_MCAL_RCC_H_

#include "rcc_reg.h"
#include "../mcal_std_types.h"


#define CLOCK_SOURCE HSI



typedef enum {
	APB2 = 0,
	APB1,
	AHB
}bus_id;

typedef enum {
	RCC_AHB_GPIO_PORTH = 16,
	RCC_AHB_GPIO_PORTA,
	RCC_AHB_GPIO_PORTB,
	RCC_AHB_GPIO_PORTC,
	RCC_AHB_GPIO_PORTD,
	RCC_AHB_GPIO_PORTE,
	RCC_AHB_GPIO_PORTF,
	RCC_AHB_GPIO_PORTG
}rcc_ahb_per_id;

typedef enum {
	RCC_APB2_SYSCFGEN = 0,

}rcc_apb2_per_id;


void RCC_voidSysClkInit(void);
void RCC_voidPerClkEn(bus_id BusId, uint32 PerId);
void RCC_voidPerClkDis(bus_id BusId, uint32 PerId);






#endif /* RCC_MCAL_RCC_H_ */
