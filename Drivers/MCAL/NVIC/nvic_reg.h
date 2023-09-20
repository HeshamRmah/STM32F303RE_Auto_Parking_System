/*
 * nvic_reg.h
 *
 *  Created on: Aug 13, 2023
 *      Author: Hesham
 */

#ifndef NVIC_NVIC_REG_H_
#define NVIC_NVIC_REG_H_


#define NVIC_BASE                         (0xE000E100)
#define NVIC_ISERx_BASE                   (NVIC_BASE + 0x000)
#define NVIC_ICERx_BASE                   (NVIC_BASE + 0x080)
#define NVIC_ISPRx_BASE                   (NVIC_BASE + 0x100)
#define NVIC_ICPRx_BASE                   (NVIC_BASE + 0x180)
#define NVIC_IABRx_BASE                   (NVIC_BASE + 0x200)
#define NVIC_IPRx_BASE                    (NVIC_BASE + 0x300)
#define NVIC_STIR_BASE                    (NVIC_BASE + 0xE00)


typedef struct {
	uint32 NVIC_ISER0;
	uint32 NVIC_ISER1;
	uint32 NVIC_ISER2;
	uint32 NVIC_ISER3;
	uint32 NVIC_ISER4;
	uint32 NVIC_ISER5;
	uint32 NVIC_ISER6;
	uint32 NVIC_ISER7;
}NVIC_ISERx_t;

typedef struct {
	uint32 NVIC_ICER0;
	uint32 NVIC_ICER1;
	uint32 NVIC_ICER2;
	uint32 NVIC_ICER3;
	uint32 NVIC_ICER4;
	uint32 NVIC_ICER5;
	uint32 NVIC_ICER6;
	uint32 NVIC_ICER7;
}NVIC_ICERx_t;

typedef struct {
	uint32 NVIC_ISPR0;
	uint32 NVIC_ISPR1;
	uint32 NVIC_ISPR2;
	uint32 NVIC_ISPR3;
	uint32 NVIC_ISPR4;
	uint32 NVIC_ISPR5;
	uint32 NVIC_ISPR6;
	uint32 NVIC_ISPR7;
}NVIC_ISPRx_t;

typedef struct {
	uint32 NVIC_ICPR0;
	uint32 NVIC_ICPR1;
	uint32 NVIC_ICPR2;
	uint32 NVIC_ICPR3;
	uint32 NVIC_ICPR4;
	uint32 NVIC_ICPR5;
	uint32 NVIC_ICPR6;
	uint32 NVIC_ICPR7;
}NVIC_ICPRx_t;

typedef struct {
	uint32 NVIC_IABR0;
	uint32 NVIC_IABR1;
	uint32 NVIC_IABR2;
	uint32 NVIC_IABR3;
	uint32 NVIC_IABR4;
	uint32 NVIC_IABR5;
	uint32 NVIC_IABR6;
	uint32 NVIC_IABR7;
}NVIC_IABRx_t;



#define  NVIC_ISERx    (*(volatile NVIC_ISERx_t *)NVIC_ISERx_BASE)
#define  NVIC_ICERx    (*(volatile NVIC_ICERx_t *)NVIC_ICERx_BASE)
#define  NVIC_ISPRx    (*(volatile NVIC_ISPRx_t *)NVIC_ISPRx_BASE)
#define  NVIC_ICPRx    (*(volatile NVIC_ICPRx_t *)NVIC_ICPRx_BASE)
#define  NVIC_IABRx    (*(volatile NVIC_IABRx_t *)NVIC_IABRx_BASE)
#define  NVIC_IPRx     ((volatile uint8 *)NVIC_IPRx_BASE)
#define  NVIC_STIR     (*(volatile uint32 *)NVIC_STIR_BASE)



#endif /* NVIC_NVIC_REG_H_ */
