/*
 * nvic.c
 *
 *  Created on: Aug 13, 2023
 *      Author: Hesham
 */

#include "nvic.h"


void NVIC_voidInit(void)
{
#ifndef SCB_AIRCR
#define SCB_AIRCR		//put address here
#endif

	AIRCR = NUM_OF_GRP_SUB; //configuration b #define fel private for example GRP_4_SUB_4
	//0x05FA0500
	//fel private #define NVIC_IPR0 ((volatile u8)(address))
	// NVIC_IPR		((volatile u8 *)address)
}
void NVIC_voidEnabeInterrupt(uint8 InterruptNum)
{
	if(InterruptNum < 32)
	{
		NVIC_ISERx.NVIC_ISER0 = 1 << InterruptNum;
	}else if(InterruptNum < 64)
	{
		InterruptNum -= 32;
		NVIC_ISERx.NVIC_ISER1 = 1 << InterruptNum;
	}
}

void NVIC_voidDisableInterrupt(uint8 InterruptNum)
{
	if(InterruptNum < 32)
	{
		NVIC_ICERx.NVIC_ICER0 = 1 << InterruptNum;
	}else if(InterruptNum < 64)
	{
		InterruptNum -= 32;
		NVIC_ICERx.NVIC_ICER1 = 1 << InterruptNum;
	}
}

void NVIC_voidSetPendingFlag(uint8 InterruptNum)
{
	if(InterruptNum < 32)
	{
		NVIC_ISPRx.NVIC_ISPR0 = 1 << InterruptNum;
	}else if(InterruptNum < 64)
	{
		InterruptNum -= 32;
		NVIC_ISPRx.NVIC_ISPR1 = 1 << InterruptNum;
	}
}


void NVIC_voidClrPendingFlag(uint8 InterruptNum)
{
	if(InterruptNum < 32)
	{
		NVIC_ICPRx.NVIC_ICPR0 = 1 << InterruptNum;
	}else if(InterruptNum < 64)
	{
		InterruptNum -= 32;
		NVIC_ICPRx.NVIC_ICPR1 = 1 << InterruptNum;
	}
}

uint8 NVIC_u8ReadActiveFlag(uint8 InterruptNum)
{
	uint8 Local_u8Acive = 0;
	if(InterruptNum < 32)
	{
		Local_u8Acive = GET_BIT(NVIC_IABRx.NVIC_IABR0,InterruptNum);
	}else if(InterruptNum < 64)
	{
		InterruptNum -= 32;
		Local_u8Acive = GET_BIT(NVIC_IABRx.NVIC_IABR1,InterruptNum);
	}
	return Local_u8Acive;
}

void NVIC_voidSetPriority(uint8 InterruptNum, uint8 priority)
{
	NVIC_IPRx[InterruptNum] = (priority << 4);
}

