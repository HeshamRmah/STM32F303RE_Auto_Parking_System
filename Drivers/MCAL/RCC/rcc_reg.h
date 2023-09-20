/*
 * rcc_reg.h
 *
 *  Created on: Aug 11, 2023
 *      Author: Hesham
 */

#ifndef RCC_RCC_REG_H_
#define RCC_RCC_REG_H_

#include "rcc_reg.h"
#include "../mcal_std_types.h"

#define AHB1_BASE     (0x40024000)

#define RCC_BASE      (0x40021000)


#define RCC_CR			*((volatile uint32 *)(RCC_BASE + 0x00))
#define RCC_CFGR		*((volatile uint32 *)(RCC_BASE + 0x04))
#define RCC_CIR			*((volatile uint32 *)(RCC_BASE + 0x08))
#define RCC_APB2RSTR	*((volatile uint32 *)(RCC_BASE + 0x0C))
#define RCC_APB1RSTR	*((volatile uint32 *)(RCC_BASE + 0x10))
#define RCC_AHBENR		*((volatile uint32 *)(RCC_BASE + 0x14))

typedef struct {
	uint32 CR;
	uint32 CFGR;
	uint32 CIR;
	uint32 APB2RSTR;
	uint32 APB1RSTR;
	uint32 AHBENR;
	uint32 APB2ENR;
	uint32 APB1ENR;
	uint32 BDCR;
	uint32 CSR;
	uint32 AHBRSTR;
	uint32 CFGR2;
	uint32 CFGR3;
}RCC_t;

#define RCC                       (*((volatile RCC_t *)RCC_BASE))















#endif /* RCC_RCC_REG_H_ */
