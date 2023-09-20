/*
 * mcal_rcc.c
 *
 *  Created on: Aug 11, 2023
 *      Author: Hesham
 */

#include "mcal_rcc.h"

void RCC_voidSysClkInit(void){
#if CLOCK_SOURCE == HSI
	SET_BIT(RCC_CR, 0);

	CLEAR_BIT(RCC_CFGR, 0);
	CLEAR_BIT(RCC_CFGR, 1);
#elif CLOCK_SOURCE == HSE
	SET_BIT(RCC_CR, 16);

	SET_BIT(RCC_CFGR, 0);
	CLEAR_BIT(RCC_CFGR, 1);
#elif CLOCK_SOURCE == PLL
	SET_BIT(RCC_CR, 24);

	CLEAR_BIT(RCC_CFGR, 0);
	SET_BIT(RCC_CFGR, 1);
#endif
}

void RCC_voidPerClkEn(bus_id BusId, uint32 PerId){
	switch(BusId){
	case AHB:
		SET_BIT(RCC.AHBENR, PerId);
		break;
	case APB1:
		SET_BIT(RCC.APB1ENR, PerId);
			break;
	case APB2:
		SET_BIT(RCC.APB2ENR, PerId);
			break;
	default:

	}
}

void RCC_voidPerClkDis(bus_id BusId, uint32 PerId){
	switch(BusId){
		case AHB:
			CLEAR_BIT(RCC.AHBENR, PerId);
			break;
		case APB1:
			CLEAR_BIT(RCC.APB1ENR, PerId);
			break;
		case APB2:
			CLEAR_BIT(RCC.APB2ENR, PerId);
			break;
		default:

		}
}

