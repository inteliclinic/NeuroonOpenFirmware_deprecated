#ifndef IC_I2C_H_
#define IC_I2C_H_

/**
 * @file
 * @brief I2C communication header
 */

#include "ic_lib_config.h"

#define TIMEOUT_VAL 			600

/**
 * @struct twi_pin_config
 * @brief I2C pins struct
 */
typedef struct {
	uint8_t gpio_sda_pin;
	uint8_t gpio_scl_pin;
} twi_pin_config;

/** CLOCK FREQUENCY CODE */
typedef enum {
	K100 = 0,
	K250,
	K400
}twi_clk_freq;

/** GENERIC CONFIGURATION FUNCTIONS */
void TWI_PIN_SELECT (uint8_t scl_pin, uint8_t sda_pin);
void TWI_Init(twi_clk_freq clk_freq);
void TWI_Deinit(void);
bool TWI_Clear_Bus(void);
bool TWI_Hard_Clear_Bus (void);
void TWI_Reset_Bus(void);

/** GENERIC WRITE/READ FUNCTIONS */
uint8_t TWI_WriteReg(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint32_t data_length);
uint8_t TWI_ReadReg(uint8_t slave_address,uint8_t register_address,uint8_t *data, uint32_t data_length);

#endif /* IC_I2C_H_ */
