/*
 * systick.c
 *
 *  Created on: Aug 14, 2023
 *      Author: Hesham
 */

#include "systick.h"



Std_ReturnType systick_intialize(const Systick_obj_t *_systick_config){
    Std_ReturnType ret = E_OK;
    if(NULL == _systick_config){
        ret = E_NOT_OK;
    }
    else{
    	SysTick.STK_VAL.value = 0;
    	SysTick.STK_LOAD.value = ((_systick_config->value) - 1);

    	switch(_systick_config->clock_source){
    	case SysTick_CLK_SCR_AHB:
    		SysTick_CLKSCR_AHB();
    		break;
    	case SysTick_CLK_SCR_AHB_DIV_BY_8:
    		SysTick_CLKSCR_AHB_DIV_BY_8();
    	    break;
    	default:
    		ret = E_NOT_OK;
    	}


    	switch(_systick_config->exception_mode){
    	    	case SysTick_POLLING_REQUEST:
    	    		SysTick_EXCEPTION_REQUEST_DISABLE();
    	    		break;
    	    	case SysTick_EXCEPTION_REQUEST:
						SysTick_EXCEPTION_REQUEST_ENABLE();
    	    	    break;
    	    	default:
    	    		ret = E_NOT_OK;
    	    	}

    }
    return ret;
}


Std_ReturnType systick_Start(const Systick_obj_t *_systick_config){
	Std_ReturnType ret = E_OK;
	    if(NULL == _systick_config){
	        ret = E_NOT_OK;
	    }
	    else{
	    	SysTick_ENABLE();

	    }
	    return ret;
}

Std_ReturnType systick_Stop(const Systick_obj_t *_systick_config){
	Std_ReturnType ret = E_OK;
		    if(NULL == _systick_config){
		        ret = E_NOT_OK;
		    }
		    else{
		    	SysTick_DISABLE();
		    }
		    return ret;
}

Std_ReturnType systick_SetTime(Systick_obj_t *_systick_config, uint32 value){
	Std_ReturnType ret = E_OK;
		    if(NULL == _systick_config){
		        ret = E_NOT_OK;
		    }
		    else{
		    	SysTick_DISABLE();
		    	_systick_config->value = (value - 1);
		    	SysTick.STK_LOAD.value = _systick_config->value;
		    	SysTick_DISABLE();
		    }
		    return ret;
}

