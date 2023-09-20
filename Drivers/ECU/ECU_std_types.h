/*
 * ECU_std_types.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Hesham
 */

#ifndef ECU_STD_TYPES_H_
#define ECU_STD_TYPES_H_

#include "stdio.h"

typedef enum
{
  ECU_OK       = 0x00U,
  ECU_ERROR    = 0x01U,
  ECU_BUSY     = 0x02U,
  ECU_TIMEOUT  = 0x03U
} ECU_StatusTypeDef;



#endif /* ECU_STD_TYPES_H_ */
