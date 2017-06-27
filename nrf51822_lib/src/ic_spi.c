/**
 * @file
 * @brief SPI communication methods
 */

#include "ic_spi.h"

#ifdef USE_IC_SPI

#ifndef USE_IC_GPIO
	#define USE_IC_GPIO
#endif
#include "ic_gpio.h"
/**
 * @fn spi_master_init()
 * @brief initialization spi bus on master mode
 */
void spi_master_init ()
{
	uint8_t output_conf = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
	                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
	                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
	                        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
	                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	uint8_t input_conf = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
			    			| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
			    			| (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
			    			| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			    			| (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->PIN_CNF[GPIO_MOSI_PIN] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_MISO_PIN] = input_conf;
	NRF_GPIO->PIN_CNF[GPIO_SCLK_PIN] = output_conf;

	SPI_MASTER->PSELSCK = GPIO_SCLK_PIN;
	SPI_MASTER->PSELMOSI = GPIO_MOSI_PIN;
	SPI_MASTER->PSELMISO = GPIO_MISO_PIN;
	SPI_MASTER->CONFIG =  (SPI_CONFIG_ORDER_MsbFirst<<SPI_CONFIG_ORDER_Pos)
						| (SPI_CONFIG_CPOL_ActiveLow<<SPI_CONFIG_CPOL_Pos)
						| (SPI_CONFIG_CPHA_Trailing<<SPI_CONFIG_CPHA_Pos);
	SPI_MASTER->FREQUENCY = SPI_SPEED<<SPI_FREQUENCY_FREQUENCY_Pos;
	SPI_MASTER->ENABLE = SPI_ENABLE_ENABLE_Enabled<<SPI_ENABLE_ENABLE_Pos;
}

/**
 * @fn spi_master_cs_init()
 * @brief initialization CS pin
 * @param pin_num: pin number to CS init
 */
void spi_master_cs_init (uint32_t pin_num)
{
	if(pin_num >32) return;

	NRF_GPIO->PIN_CNF[pin_num] |= GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;
	SetBit(pin_num);
}

/**
 * @fn spi_master_cs_low()
 * @brief set low state on CS pin
 * @param pin_num: pin number
 */
void spi_master_cs_low(uint32_t pin_num)
{
	if(pin_num >32) return;
	ResetBit(pin_num);
}

/**
 * @fn spi_master_cs_high()
 * @brief set high state on CS pin
 * @param pin_num: pin number
 */
void spi_master_cs_high(uint32_t pin_num)
{
	if(pin_num >32) return;
	SetBit(pin_num);
}

/**
 * @fn spi_master_deinit()
 * @brief deinitialization spi bus on master mode
 */
void spi_master_deinit ()
{
	SPI_MASTER->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;

	uint8_t output_conf = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
	    	    		| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
	    	    		| (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
	    	    		| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
	    	    		| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	NRF_GPIO->PIN_CNF[GPIO_MOSI_PIN] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_SCLK_PIN] = output_conf;
	NRF_GPIO->PIN_CNF[GPIO_MISO_PIN] = 0;

	NRF_GPIO->OUTSET = 1<<GPIO_MOSI_PIN;
	NRF_GPIO->OUTSET = 1<<GPIO_SCLK_PIN;
}

/**
 * @fn spi_read_data(uint8_t reg_address, uint8_t *data)
 * @brief read value from device register
 * @param reg_address: register_address
 * @param data: pointer to the variable when data are read
 * @return 0 if data is read
 */
uint8_t spi_read_data(uint8_t reg_address, uint8_t* data)
{
	SPI_MASTER->EVENTS_READY = 0;
	SPI_MASTER->TXD = reg_address;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
	SPI_MASTER->EVENTS_READY = 0;
	*data = SPI_MASTER->RXD;

	SPI_MASTER->TXD = 0xFF;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
	SPI_MASTER->EVENTS_READY = 0;
	*data = SPI_MASTER->RXD;

	return 0;
}

/**
 * @fn spi_multipleRead_data(uint8_t reg_address, uint8_t *data, uint8_t data_length)
 * @brief read values from device registers
 * @param reg_address: register_address
 * @param data: pointer to the variable when data are read
 * @param data_length: (in byte)
 * @return 0 if data is read
 */
uint8_t spi_multipleRead_data(uint8_t reg_address, uint8_t* data, uint32_t data_length)
{
	SPI_MASTER->EVENTS_READY = 0;
	SPI_MASTER->TXD = reg_address;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
	SPI_MASTER->EVENTS_READY = 0;
	*data = SPI_MASTER->RXD;

	while(data_length-- != 0){
		SPI_MASTER->TXD = 0xFF;
		while(SPI_MASTER->EVENTS_READY != 1) __NOP();
		SPI_MASTER->EVENTS_READY = 0;
		*data++ = SPI_MASTER->RXD;
	}
		return 0;
}

static uint8_t spi_flush_rxd()
{
  return SPI_MASTER->RXD;
}

/**
 * @fn spi_write_data(uint8_t reg_address, uint8_t *data)
 * @brief write value to device register
 * @param reg_address: register_address
 * @param data: pointer to the data to write
 * @return 0 if data is write
 */
uint8_t spi_write_data(uint8_t reg_address, uint8_t data)
{
	SPI_MASTER->EVENTS_READY = 0;
	SPI_MASTER->TXD = reg_address;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
	SPI_MASTER->EVENTS_READY = 0;
        spi_flush_rxd();

	SPI_MASTER->TXD = data;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
	SPI_MASTER->EVENTS_READY = 0;
        spi_flush_rxd();

	return 0;
}

/**
 * @fn spi_write_byte(uint8_t data)
 * @brief write value to device register
 * @param data: byte to write
 * @return 0 if byte is read
 */
uint8_t spi_write_byte(uint8_t byte)
{
	SPI_MASTER->EVENTS_READY = 0;
	SPI_MASTER->TXD = byte;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
        spi_flush_rxd();

	return 0;
}

/**
 * @fn spi_multipleWrite_data(uint8_t reg_address, uint8_t *data, uint8_t data_length)
 * @brief write values to device registers
 * @param reg_address: register_address
 * @param data: pointer to the data to write
 * @param data_length: (in byte)
 * @return 0 if data is write
 */
uint8_t spi_multipleWrite_data(uint8_t reg_address, uint8_t* data, uint32_t data_length)
{
	uint32_t i;

	SPI_MASTER->EVENTS_READY = 0;
	SPI_MASTER->TXD = reg_address;
	while(SPI_MASTER->EVENTS_READY != 1) __NOP();
	SPI_MASTER->EVENTS_READY = 0;
        spi_flush_rxd();

	for(i = 0; i < data_length; i++){
		SPI_MASTER->TXD = data[i];
		while(SPI_MASTER->EVENTS_READY != 1) __NOP();
		SPI_MASTER->EVENTS_READY = 0;
                spi_flush_rxd();
	}

        return 0;
}
#endif
