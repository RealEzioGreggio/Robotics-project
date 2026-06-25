/*
 * new_printf.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lorenzo
 */

#include "stm32f4xx.h"

int _write(int file, char *ptr, int len){

	for(int DataIdx = 0; DataIdx < len; DataIdx++){

		ITM_SendChar(*ptr++);
	}
	return len;
}
