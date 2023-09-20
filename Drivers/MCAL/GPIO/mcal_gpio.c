/*
 * mcal_gpio.c
 *
 *  Created on: Aug 12, 2023
 *      Author: Hesham
 */


#include "mcal_gpio.h"

/* Arrays to Concatonate GPIO Registers ...
volatile uint32 *MODER_registers[] = {&GPIOA.GPIOx_MODER, &GPIOB.GPIOx_MODER, &GPIOC.GPIOx_MODER,
										&GPIOD.GPIOx_MODER, &GPIOE.GPIOx_MODER, &GPIOF.GPIOx_MODER,
										&GPIOG.GPIOx_MODER, &GPIOH.GPIOx_MODER};

volatile uint32 *OTYPER_registers[] = {&GPIOA.GPIOx_OTYPER, &GPIOB.GPIOx_OTYPER, &GPIOC.GPIOx_OTYPER,
										&GPIOD.GPIOx_OTYPER, &GPIOE.GPIOx_OTYPER, &GPIOF.GPIOx_OTYPER,
										&GPIOG.GPIOx_OTYPER, &GPIOH.GPIOx_OTYPER};

volatile uint32 *OSPEEDR_registers[] = {&GPIOA.GPIOx_OSPEEDR, &GPIOB.GPIOx_OSPEEDR, &GPIOC.GPIOx_OSPEEDR,
										&GPIOD.GPIOx_OSPEEDR, &GPIOE.GPIOx_OSPEEDR, &GPIOF.GPIOx_OSPEEDR,
										&GPIOG.GPIOx_OSPEEDR, &GPIOH.GPIOx_OSPEEDR};

volatile uint32 *PUPDR_registers[] = {&GPIOA.GPIOx_PUPDR, &GPIOB.GPIOx_PUPDR, &GPIOC.GPIOx_PUPDR,
										&GPIOD.GPIOx_PUPDR, &GPIOE.GPIOx_PUPDR, &GPIOF.GPIOx_PUPDR,
										&GPIOG.GPIOx_PUPDR, &GPIOH.GPIOx_PUPDR};

volatile uint32 *IDR_registers[] = {&GPIOA.GPIOx_IDR, &GPIOB.GPIOx_IDR, &GPIOC.GPIOx_IDR,
										&GPIOD.GPIOx_IDR, &GPIOE.GPIOx_IDR, &GPIOF.GPIOx_IDR,
										&GPIOG.GPIOx_IDR, &GPIOH.GPIOx_IDR};

volatile uint32 *ODR_registers[] = {&GPIOA.GPIOx_ODR, &GPIOB.GPIOx_ODR, &GPIOC.GPIOx_ODR,
										&GPIOD.GPIOx_ODR, &GPIOE.GPIOx_ODR, &GPIOF.GPIOx_ODR,
										&GPIOG.GPIOx_ODR, &GPIOH.GPIOx_ODR};

volatile uint32 *BSRR_registers[] = {&GPIOA.GPIOx_BSRR, &GPIOB.GPIOx_BSRR, &GPIOC.GPIOx_BSRR,
										&GPIOD.GPIOx_BSRR, &GPIOE.GPIOx_BSRR, &GPIOF.GPIOx_BSRR,
										&GPIOG.GPIOx_BSRR, &GPIOH.GPIOx_BSRR};

volatile uint32 *LCKR_registers[] = {&GPIOA.GPIOx_LCKR, &GPIOB.GPIOx_LCKR, &GPIOC.GPIOx_LCKR,
										&GPIOD.GPIOx_LCKR, &GPIOE.GPIOx_LCKR, &GPIOF.GPIOx_LCKR,
										&GPIOG.GPIOx_LCKR, &GPIOH.GPIOx_LCKR};

volatile uint32 *AFRL_registers[] = {&GPIOA.GPIOx_AFRL, &GPIOB.GPIOx_AFRL, &GPIOC.GPIOx_AFRL,
										&GPIOD.GPIOx_AFRL, &GPIOE.GPIOx_AFRL, &GPIOF.GPIOx_AFRL,
										&GPIOG.GPIOx_AFRL, &GPIOH.GPIOx_AFRL};

volatile uint32 *AFRH_registers[] = {&GPIOA.GPIOx_AFRH, &GPIOB.GPIOx_AFRH, &GPIOC.GPIOx_AFRH,
										&GPIOD.GPIOx_AFRH, &GPIOE.GPIOx_AFRH, &GPIOF.GPIOx_AFRH,
										&GPIOG.GPIOx_AFRH, &GPIOH.GPIOx_AFRH};

volatile uint32 *BRR_registers[] = {&GPIOA.GPIOx_BRR, &GPIOB.GPIOx_BRR, &GPIOC.GPIOx_BRR,
										&GPIOD.GPIOx_BRR, &GPIOE.GPIOx_BRR, &GPIOF.GPIOx_BRR,
										&GPIOG.GPIOx_BRR, &GPIOH.GPIOx_BRR};
*/


Std_ReturnType gpio_pin_mode_intialize(const pin_config_t *_pin_config){
    Std_ReturnType ret = E_OK;
    if(NULL == _pin_config || _pin_config->pin > PORT_PIN_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{

        switch(_pin_config->mode){
            case GPIO_OUTPUT_PUSHPULL_LOWSPEED :

            	//SET_BIT(*MODER_registers[_pin_config->port],(_pin_config->pin * 2));
            	//CLEAR_BIT(*MODER_registers[_pin_config->port],(_pin_config->pin * 2 + 1));
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2 + 1));

            	//CLEAR_BIT(*OTYPER_registers[_pin_config->port],_pin_config->pin);
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_OTYPER,_pin_config->pin);

            	//CLEAR_BIT(*OSPEEDR_registers[_pin_config->port],(_pin_config->pin * 2));
            	//CLEAR_BIT(*OSPEEDR_registers[_pin_config->port],(_pin_config->pin * 2 + 1));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_OSPEEDR,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_OSPEEDR,(_pin_config->pin * 2 + 1));


                break;
            case GPIO_OUTPUT_OPENDRAIN_LOWSPEED :
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2 + 1));

            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_OTYPER,_pin_config->pin);

            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_OSPEEDR,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_OSPEEDR,(_pin_config->pin * 2 + 1));
                break;
            case GPIO_INPUT_PULLUP :
            	//CLEAR_BIT(*MODER_registers[_pin_config->port],(_pin_config->pin * 2));
            	//CLEAR_BIT(*MODER_registers[_pin_config->port],(_pin_config->pin * 2 + 1));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2 + 1));

            	//SET_BIT(*PUPDR_registers[_pin_config->port],(_pin_config->pin * 2));
            	//CLEAR_BIT(*PUPDR_registers[_pin_config->port],(_pin_config->pin * 2 + 1));
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_PUPDR,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_PUPDR,(_pin_config->pin * 2 + 1));
                break;
            case GPIO_INPUT_PULLDOWN :
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2));
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2 + 1));

            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_PUPDR,(_pin_config->pin * 2));
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_PUPDR,(_pin_config->pin * 2 + 1));
                break;
            case GPIO_ALTERNATE :
            	CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2));
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2 + 1));
                break;
            case GPIO_ANALOG :
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2));
            	SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_MODER,(_pin_config->pin * 2 + 1));
                break;
            default : ret = E_NOT_OK;
        }
    }
    return ret;
}


/**
 *
 * @param _pin_config pointer to the configurations @ref pin_config_t
 * @param direction_status
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_pin_get_direction_status(const pin_config_t *_pin_config, uint32 *direction_status){
    Std_ReturnType ret = E_OK;
    if(NULL == _pin_config || NULL == direction_status || _pin_config->pin > PORT_PIN_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        //*direction_status = READ_BIT(*IDR_registers[_pin_config->port], _pin_config->pin);
        *direction_status = READ_BIT_h(GPIO_h[_pin_config->port].GPIOx_IDR, _pin_config->pin);
    }
    return ret;
}


/**
 *
 * @param _pin_config pointer to the configurations @ref pin_config_t
 * @param logic
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_pin_write_logic(const pin_config_t *_pin_config, gpio_logic_t logic){
    Std_ReturnType ret = E_OK;
    if(NULL == _pin_config || _pin_config->pin > PORT_PIN_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        switch(logic){
            case GPIO_LOW :
                //CLEAR_BIT(*ODR_registers[_pin_config->port], _pin_config->pin);
                CLEAR_BIT_h(GPIO_h[_pin_config->port].GPIOx_ODR, _pin_config->pin);
                break;
            case GPIO_HIGH :
                //SET_BIT(*ODR_registers[_pin_config->port], _pin_config->pin);
                SET_BIT_h(GPIO_h[_pin_config->port].GPIOx_ODR, _pin_config->pin);
                break;
            default : ret = E_NOT_OK;
        }
    }
    return ret;
}


/**
 *
 * @param _pin_config pointer to the configurations @ref pin_config_t
 * @param logic
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_pin_read_logic(const pin_config_t *_pin_config, gpio_logic_t *logic){
    Std_ReturnType ret = E_OK;
    if(NULL == _pin_config || NULL == logic || _pin_config->pin > PORT_PIN_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        //*logic = READ_BIT(*IDR_registers[_pin_config->port], _pin_config->pin);
        *logic = READ_BIT_h(GPIO_h[_pin_config->port].GPIOx_IDR, _pin_config->pin);
    }
    return ret;
}


/**
 *
 * @param _pin_config pointer to the configurations @ref pin_config_t
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_pin_toggle_logic(const pin_config_t *_pin_config){
    Std_ReturnType ret = E_OK;
    if(NULL == _pin_config || _pin_config->pin > PORT_PIN_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        TOGGLE_BIT_h(GPIO_h[_pin_config->port].GPIOx_ODR, _pin_config->pin);
    }
    return ret;
}


/**
 *
 * @param _pin_config pointer to the configurations @ref pin_config_t
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_pin_intialize(const pin_config_t *_pin_config){
    Std_ReturnType ret = E_OK;
    if(NULL == _pin_config || _pin_config->pin > PORT_PIN_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        ret = gpio_pin_mode_intialize(_pin_config);
        ret = gpio_pin_write_logic(_pin_config, _pin_config->logic);
    }
    return ret;
}


/**
 *
 * @param port
 * @param direction
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_port_direction_intialize(port_index_t port, uint8 direction){
    Std_ReturnType ret = E_OK;
    if(port > PORT_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        //*tris_registers[port] = direction;
    }
    return ret;
}


/**
 *
 * @param port
 * @param direction_status
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_port_get_direction_status(port_index_t port, uint8 *direction_status){
    Std_ReturnType ret = E_OK;
    if((NULL == direction_status) && (port > PORT_MAX_NUMBER-1)){
        ret = E_NOT_OK;
    }
    else{
        //*direction_status = *tris_registers[port];
    }
    return ret;
}


/**
 *
 * @param port
 * @param logic
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_port_write_logic(port_index_t port, uint8 logic){
    Std_ReturnType ret = E_OK;
    if(port > PORT_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        //*lat_registers[port] = logic;
    }
    return ret;
}


/**
 *
 * @param port
 * @param logic
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_port_read_logic(port_index_t port, uint8 *logic){
    Std_ReturnType ret = E_OK;
    if((NULL == logic) && (port > PORT_MAX_NUMBER-1)){
        ret = E_NOT_OK;
    }
    else{
        //*logic = *lat_registers[port];
    }
    return ret;
}


/**
 *
 * @param port
 * @return Status of the function
 *          (E_OK) : The function done successfully
 *          (E_NOT_OK) : The function has issue to perform this action
 */

Std_ReturnType gpio_port_toggle_logic(port_index_t port){
    Std_ReturnType ret = E_OK;
    if(port > PORT_MAX_NUMBER-1){
        ret = E_NOT_OK;
    }
    else{
        //*lat_registers[port] ^= PORTC_MASK;
    }
    return ret;
}


