/**
 * @file
 * @brief delay in ms methods
 */

#include "ic_delay.h"
#ifdef USE_IC_DELAY
/**
 * @fn ic_delay_ms ()
 * @brief delay in ms
 * @param delay time in miliseconds
 */
void ic_delay_ms(uint32_t ms)
{
	uint32_t us = ms * 1000;

	for(;us > 0; us--){
		//tyle wychodzi na oscylu
		asm("MOVS	R2, R2");
		asm("MOVS	R2, R2");
		asm("MOVS	R2, R2");
		asm("MOVS	R2, R2");
		asm("MOVS	R2, R2");
	}
}
#endif
