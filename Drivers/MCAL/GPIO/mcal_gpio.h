/*
 * mcal_gpio.h
 *
 *  Created on: Aug 12, 2023
 *      Author: Hesham
 */

#ifndef GPIO_MCAL_GPIO_H_
#define GPIO_MCAL_GPIO_H_

#include "gpio_reg.h"
#include "../mcal_std_types.h"


#define BIT_MASK  (uint32)1
#define PORT_PIN_MAX_NUMBER  16
#define PORT_MAX_NUMBER      8

typedef enum{
    GPIO_LOW = 0,
    GPIO_HIGH
}gpio_logic_t;

typedef enum{
    GPIO_OUTPUT_PUSHPULL_LOWSPEED = 0,
	GPIO_OUTPUT_PUSHPULL_MEDIUMSPEED,
	GPIO_OUTPUT_PUSHPULL_HIGHSPEED,
	GPIO_OUTPUT_OPENDRAIN_LOWSPEED,
	GPIO_OUTPUT_OPENDRAIN_MEDIUMSPEED,
	GPIO_OUTPUT_OPENDRAIN_HIGHSPEED,
    GPIO_INPUT_PULLUP,
	GPIO_INPUT_PULLDOWN,
	GPIO_ALTERNATE,
	GPIO_ANALOG,
}gpio_mode_t;

typedef enum{
    GPIO_PIN0 = 0,
    GPIO_PIN1,
    GPIO_PIN2,
    GPIO_PIN3,
    GPIO_PIN4,
    GPIO_PIN5,
    GPIO_PIN6,
    GPIO_PIN7,
	GPIO_PIN8,
	GPIO_PIN9,
	GPIO_PIN10,
	GPIO_PIN11,
	GPIO_PIN12,
	GPIO_PIN13,
	GPIO_PIN14,
	GPIO_PIN15
}pin_index_t;

typedef enum{
    PORTA_INDEX = 0,
    PORTB_INDEX,
    PORTC_INDEX,
    PORTD_INDEX,
    PORTE_INDEX,
	PORTF_INDEX,
	PORTG_INDEX,
	PORTH_INDEX,
}port_index_t;

typedef struct{
	port_index_t port;     /* @ref port_index_t */
	pin_index_t pin;       /* @ref pin_index_t */
	gpio_mode_t mode;      /* @ref gpio_mode_t */
	gpio_logic_t logic;    /* @ref gpio_logic_t */
}pin_config_t;

/* Section: Function Declarations */
Std_ReturnType gpio_pin_mode_intialize(const pin_config_t *_pin_config);
Std_ReturnType gpio_pin_get_direction_status(const pin_config_t *_pin_config, uint32 *direction_status);
Std_ReturnType gpio_pin_write_logic(const pin_config_t *_pin_config, gpio_logic_t logic);
Std_ReturnType gpio_pin_read_logic(const pin_config_t *_pin_config, gpio_logic_t *logic);
Std_ReturnType gpio_pin_toggle_logic(const pin_config_t *_pin_config);
Std_ReturnType gpio_pin_intialize(const pin_config_t *_pin_config);

Std_ReturnType gpio_port_direction_intialize(port_index_t port, uint8 direction);
Std_ReturnType gpio_port_get_direction_status(port_index_t port, uint8 *direction_status);
Std_ReturnType gpio_port_write_logic(port_index_t port, uint8 logic);
Std_ReturnType gpio_port_read_logic(port_index_t port, uint8 *logic);
Std_ReturnType gpio_port_toggle_logic(port_index_t port);

#endif /* GPIO_MCAL_GPIO_H_ */
