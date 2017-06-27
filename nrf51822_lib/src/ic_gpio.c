/*
 * ic_gpio.c
 *
 *  Created on: May 14, 2013
 *      Author: krzysiek
 */

#include "ic_gpio.h"
#ifdef USE_IC_GPIO

/**
 * @file
 * @brief GPIO methods
 */

/**
 * @fn SetBit(uint32_t pin)
 * @brief Set bit to '1'
 * @param pin GPIO pin number
 */
void SetBit(uint32_t pin) {
	NRF_GPIO->OUTSET = 1 << pin;
}

/**
 * @fn SetBitMask(uint32_t mask)
 * @brief Set multiple bits to '1' 
 * @param mask selected bits binary mask
 */
void SetBitMask(uint32_t mask){
	NRF_GPIO->OUTSET = mask;
}

/**
 * @fn ResetBit(uint32_t pin)
 * @brief Reset bit to '0'
 * @param pin GPIO pin number
 */
void ResetBit(uint32_t pin) {
	NRF_GPIO->OUTCLR = 1 << pin;
}

/**
 * @fn ResetBitMask(uint32_t mask)
 * @brief Reset multiple bits to '0' 
 * @param mask selected bits binary mask
 */
void ResetBitMask(uint32_t mask){
	NRF_GPIO->OUTCLR = mask;
}

/**
 * @fn ChangeBit(uint32_t pin)
 * @brief Change pin value
 * @param pin GPIO pin number
 */
void ChangeBit(uint32_t pin) {
	if(NRF_GPIO->OUT & (1 << pin))
		NRF_GPIO->OUTCLR = 1 << pin;
	else
		NRF_GPIO->OUTSET = 1 << pin;
}
#endif
