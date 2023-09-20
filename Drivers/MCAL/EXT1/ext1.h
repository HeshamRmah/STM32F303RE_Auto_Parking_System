/*
 * ext1.h
 *
 *  Created on: Aug 13, 2023
 *      Author: Hesham
 */

#ifndef EXT1_EXT1_H_
#define EXT1_EXT1_H_

#include "../mcal_std_types.h"
#include "ext1_reg.h"


/*********************************************************/
/*OPTIONS : 1- LINE0_PA0
            2- LINE0_PB0
            3- LINE0_PC0 */

#define EXTI0 LINE0_PA0
/********************************************************/
/*OPTIONS : 1- LINE1_PA1
            2- LINE1_PB1
            3- LINE1_PC1 */

#define EXTI1  LINE1_PA1
/********************************************************/
/*OPTIONS : 1- LINE2_PA2
            2- LINE2_PB2
            3- LINE2_PC2 */

#define EXTI2  LINE2_PA2
/********************************************************/
/*OPTIONS : 1- LINE3_PA3
            2- LINE3_PB3
            3- LINE3_PC3 */

#define EXTI3   LINE3_PA3
/********************************************************/
/*OPTIONS : 1- LINE4_PA4
            2- LINE4_PB4
            3- LINE4_PC4 */

#define EXTI4    LINE4_PA4
/********************************************************/
/*OPTIONS : 1- LINE5_PA5
            2- LINE5_PB5
            3- LINE5_PC5 */

#define EXTI5     LINE5_PA5
/********************************************************/
/*OPTIONS : 1- LINE6_PA6
            2- LINE6_PB6
            3- LINE6_PC6 */


#define EXTI6    LINE6_PA6
/********************************************************/
/*OPTIONS : 1- LINE7_PA7
            2- LINE7_PB7
            3- LINE7_PC7 */

#define EXTI7   LINE7_PA7
/********************************************************/
/*OPTIONS : 1- LINE8_PA8
            2- LINE8_PB8
            3- LINE8_PC8 */


#define EXTI8    LINE8_PA8
/********************************************************/
/*OPTIONS : 1- LINE9_PA9
            2- LINE9_PB9
            3- LINE9_PC9 */


#define EXTI9    LINE9_PA9
/********************************************************/
/*OPTIONS : 1- LINE10_PA10
            2- LINE10_PB10
            3- LINE10_PC10 */


#define EXTI10  LINE10_PA10
/********************************************************/
/*OPTIONS : 1- LINE11_PA11
            2- LINE11_PB11
            3- LINE11_PC11 */


#define EXTI11   LINE11_PA11
/********************************************************/
/*OPTIONS : 1- LINE12_PA12
            2- LINE12_PB12
            3- LINE12_PC12 */


#define EXTI12     LINE12_PA12
/********************************************************/
/*OPTIONS : 1- LINE13_PA13
            2- LINE13_PB13
            3- LINE13_PC13 */


#define EXTI13     LINE13_PC13
/********************************************************/
/*OPTIONS : 1- LINE14_PA14
            2- LINE14_PB14
            3- LINE14_PC14 */


#define EXTI14   LINE14_PA14
/********************************************************/
/*OPTIONS : 1- LINE15_PA15
            2- LINE15_PB15
            3- LINE15_PC15 */


#define EXTI15   LINE15_PA15
/********************************************************/




#define EXTI_LINE_0       0
#define EXTI_LINE_1       1
#define EXTI_LINE_2       2
#define EXTI_LINE_3       3
#define EXTI_LINE_4       4
#define EXTI_LINE_5       5
#define EXTI_LINE_6       6
#define EXTI_LINE_7       7
#define EXTI_LINE_8       8
#define EXTI_LINE_9       9
#define EXTI_LINE_10      10
#define EXTI_LINE_11      11
#define EXTI_LINE_12      12
#define EXTI_LINE_13      13
#define EXTI_LINE_14      14
#define EXTI_LINE_15      15

#define RISING_TRIGGER      0
#define FALLING_TRIGGER     1
#define ON_CHANGE_TRIGGER   2


void EXTI_Init(void);
void EXTI_voidSetSenseMood (uint8 Copy_u8Line_Num ,uint8 Copy_u8SenseMood);
void EXTI_voidEnableLine   (uint8 Copy_u8Line_Num);
void EXTI_voidDisableLine  (uint8 Copy_u8Line_Num);


#endif /* EXT1_EXT1_H_ */
