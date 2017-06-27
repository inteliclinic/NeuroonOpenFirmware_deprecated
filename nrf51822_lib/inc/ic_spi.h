#ifndef __IC_SPI__H
#define __IC_SPI__H

/**
 * @file
 * @brief SPI communication header
 */

#include "ic_lib_config.h"

/** GENERIC CONFIGURATION FUNCTIONS */
void spi_master_init(void);
void spi_master_deinit(void);
void spi_master_cs_low(uint32_t pin_num);
void spi_master_cs_high(uint32_t pin_num);
void spi_master_cs_init (uint32_t pin_num);

/** GENERIC WRITE/READ FUNCTIONS */
uint8_t spi_read_data(uint8_t reg_address, uint8_t* data);
uint8_t spi_multipleRead_data(uint8_t reg_address, uint8_t* data, uint32_t data_length);
uint8_t spi_write_data(uint8_t reg_address, uint8_t data);
uint8_t spi_write_byte(uint8_t byte);
uint8_t spi_multipleWrite_data(uint8_t reg_address, uint8_t* data, uint32_t data_length);


#endif
