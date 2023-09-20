/*
 * scb_reg.h
 *
 *  Created on: Aug 13, 2023
 *      Author: Hesham
 */

#ifndef SCB_SCB_REG_H_
#define SCB_SCB_REG_H_

#define AIRCR_BASE                    (0xE000ED0C)

#define AIRCR                      (*(volatile uint32 *)AIRCR_BASE)

#endif /* SCB_SCB_REG_H_ */
