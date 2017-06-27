
#ifndef IC_GPIO_H_
#define IC_GPIO_H_

#include "nrf.h"
#include "ic_lib_config.h"

/**
 * @file
 * @brief GPIO methods header file
 */
 
void SetBit(uint32_t pin);
void ResetBit(uint32_t pin);
void ChangeBit(uint32_t pin);
void SetBitMask(uint32_t mask);
void ResetBitMask(uint32_t mask);

#endif
