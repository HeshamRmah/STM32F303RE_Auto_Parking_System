/*
 * ext1_reg.h
 *
 *  Created on: Aug 13, 2023
 *      Author: Hesham
 */

#ifndef EXT1_EXT1_REG_H_
#define EXT1_EXT1_REG_H_

#include "ext1_reg.h"
#include "../mcal_std_types.h"




typedef struct{
	volatile uint32 IMR1;
	volatile uint32 EMR1;
	volatile uint32 RTSR1;
	volatile uint32 FTSR1;
	volatile uint32 SWIER1;
	volatile uint32 PR1;
}EXTI_TYPE;

#define EXTI  ((EXTI_TYPE *)0x40010400)


#define LINE0_PA0    0
#define LINE0_PB0    1
#define LINE0_PC0    2

#define LINE1_PA1    3
#define LINE1_PB1    4
#define LINE1_PC1    5

#define LINE2_PA2    6
#define LINE2_PB2    7
#define LINE2_PC2    8

#define LINE3_PA3    9
#define LINE3_PB3    10
#define LINE3_PC3    11

#define LINE4_PA4    12
#define LINE4_PB4    13
#define LINE4_PC4    14

#define LINE5_PA5    15
#define LINE5_PB5    16
#define LINE5_PC5    17

#define LINE6_PA6    18
#define LINE6_PB6    19
#define LINE6_PC6    20

#define LINE7_PA7    21
#define LINE7_PB7    22
#define LINE7_PC7    23

#define LINE8_PA8    24
#define LINE8_PB8    25
#define LINE8_PC8    26

#define LINE9_PA9    27
#define LINE9_PB9    28
#define LINE9_PC9    29

#define LINE10_PA10    30
#define LINE10_PB10    31
#define LINE10_PC10    32

#define LINE11_PA11    33
#define LINE11_PB11    34
#define LINE11_PC11    35

#define LINE12_PA12    36
#define LINE12_PB12    37
#define LINE12_PC12    38

#define LINE13_PA13    39
#define LINE13_PB13    40
#define LINE13_PC13    41

#define LINE14_PA14    42
#define LINE14_PB14    43
#define LINE14_PC14    44

#define LINE15_PA15    45
#define LINE15_PB15    46
#define LINE15_PC15    48


#endif /* EXT1_EXT1_REG_H_ */
