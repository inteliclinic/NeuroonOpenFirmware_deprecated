#ifndef __IC_UART_H_
#define __IC_UART_H_

/**
 * @file
 * @brief UART communication header
 */
#include "ic_lib_config.h"

/** GENERIC CONFIGURATION FUNCTIONS */
void uart_init(void);

/** GENERIC WRITE/READ FUNCTIONS */
void uart_send_u8(uint8_t data);
void uart_send_tab(uint32_t len, uint8_t* data);
void uart_send_rtab(uint8_t len, uint8_t* data);

#endif
