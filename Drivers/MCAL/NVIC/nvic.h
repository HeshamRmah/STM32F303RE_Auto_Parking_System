/*
 * nvic.h
 *
 *  Created on: Aug 13, 2023
 *      Author: Hesham
 */

#ifndef NVIC_NVIC_H_
#define NVIC_NVIC_H_

#include "../mcal_std_types.h"
#include "nvic_reg.h"
#include "../SCB/scb_reg.h"


#define  GROUP_16_SUBBRIORITY_0      0x05FA0300      //4 Bit For Group And 0 Bit For SubPriority
#define  GROUP_8_SUBBRIORITY_2       0x05FA0400      //3 Bit For Group And 1 Bit For SubPriority
#define  GROUP_4_SUBBRIORITY_4       0x05FA0500      //2 Bit For Group And 2 Bit For SubPriority
#define  GROUP_2_SUBBRIORITY_8       0x05FA0600      //1 Bit For Group And 3 Bit For SubPriority
#define  GROUP_0_SUBBRIORITY_16      0x05FA0700     //0 Bit For Group And 4 Bit For SubPriority

#define NUM_OF_GRP_SUB GROUP_4_SUBBRIORITY_4


void NVIC_voidInit(void);
void NVIC_voidEnabeInterrupt(uint8 InterruptNum);
void NVIC_voidDisableInterrupt(uint8 InterruptNum);
void NVIC_voidSetPendingFlag(uint8 InterruptNum);
void NVIC_voidClrPendingFlag(uint8 InterruptNum);
uint8 NVIC_u8ActiveFlag(uint8 InterruptNum);
void NVIC_voidSetPriority(uint8 InterruptNum,uint8 priority);

#endif /* NVIC_NVIC_H_ */
