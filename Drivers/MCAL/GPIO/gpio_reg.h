/*
 * gpio_reg.h
 *
 *  Created on: Aug 12, 2023
 *      Author: Hesham
 */

#ifndef GPIO_GPIO_REG_H_
#define GPIO_GPIO_REG_H_

#include "../mcal_std_types.h"

#define GPIOA_BASE_h             (0x48000000)
#define GPIOB_BASE_h             (0x48000400)
#define GPIOC_BASE_h             (0x48000800)
#define GPIOD_BASE_h             (0x48000C00)
#define GPIOE_BASE_h             (0x48001000)
#define GPIOF_BASE_h             (0x48001400)
#define GPIOG_BASE_h             (0x48001800)
#define GPIOH_BASE_h             (0x48001C00)

typedef struct {
	uint32 PIN0         :1;
	uint32 PIN1         :1;
	uint32 PIN2         :1;
	uint32 PIN3         :1;
	uint32 PIN4         :1;
	uint32 PIN5         :1;
	uint32 PIN6         :1;
	uint32 PIN7         :1;
	uint32 PIN8         :1;
	uint32 PIN9         :1;
	uint32 PIN10        :1;
	uint32 PIN11        :1;
	uint32 PIN12        :1;
	uint32 PIN13        :1;
	uint32 PIN14        :1;
	uint32 PIN15        :1;
	uint32 PIN16        :1;
	uint32 PIN17        :1;
	uint32 PIN18        :1;
	uint32 PIN19        :1;
	uint32 PIN20        :1;
	uint32 PIN21        :1;
	uint32 PIN22        :1;
	uint32 PIN23        :1;
	uint32 PIN24        :1;
	uint32 PIN25        :1;
	uint32 PIN26        :1;
	uint32 PIN27        :1;
	uint32 PIN28        :1;
	uint32 PIN29        :1;
	uint32 PIN30        :1;
	uint32 PIN31        :1;
}gpio_pins_t;

typedef struct {
	uint32 MODER0         :2;
	uint32 MODER1         :2;
	uint32 MODER2         :2;
	uint32 MODER3         :2;
	uint32 MODER4         :2;
	uint32 MODER5         :2;
	uint32 MODER6         :2;
	uint32 MODER7         :2;
	uint32 MODER8         :2;
	uint32 MODER9         :2;
	uint32 MODER10        :2;
	uint32 MODER11        :2;
	uint32 MODER12        :2;
	uint32 MODER13        :2;
	uint32 MODER14        :2;
	uint32 MODER15        :2;
}gpio_moder_pins_t;

typedef struct {
	uint32 GPIOx_MODER;
	uint32 GPIOx_OTYPER;
	uint32 GPIOx_OSPEEDR;
	uint32 GPIOx_PUPDR;
	uint32 GPIOx_IDR;
	uint32 GPIOx_ODR;
	uint32 GPIOx_BSRR;
	uint32 GPIOx_LCKR;
	uint32 GPIOx_AFRL;
	uint32 GPIOx_AFRH;
	uint32 GPIOx_BRR;
}gpio_registers_t;


typedef struct {
	gpio_registers_t GPIO_PORT_A;
	gpio_registers_t GPIO_PORT_B;
	gpio_registers_t GPIO_PORT_C;
	gpio_registers_t GPIO_PORT_D;
	gpio_registers_t GPIO_PORT_E;
	gpio_registers_t GPIO_PORT_F;
	gpio_registers_t GPIO_PORT_G;
	gpio_registers_t GPIO_PORT_H;
}GPIO_t;



#define GPIO_h  ((volatile gpio_registers_t *)GPIOA_BASE_h)

#define GPIOA_h (*(volatile gpio_registers_t *)GPIOA_BASE_h)
#define GPIOB_h (*(volatile gpio_registers_t *)GPIOB_BASE_h)
#define GPIOC_h (*(volatile gpio_registers_t *)GPIOC_BASE_h)
#define GPIOD_h (*(volatile gpio_registers_t *)GPIOD_BASE_h)
#define GPIOE_h (*(volatile gpio_registers_t *)GPIOE_BASE)h)
#define GPIOF_h (*(volatile gpio_registers_t *)GPIOF_BASE_h)
#define GPIOG_h (*(volatile gpio_registers_t *)GPIOG_BASE_h)
#define GPIOH_h (*(volatile gpio_registers_t *)GPIOH_BASE_h)








#endif /* GPIO_GPIO_REG_H_ */
