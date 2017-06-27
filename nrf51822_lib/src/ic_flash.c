/**
 * @file
 * @brief Flash memory (internal and external - EN25QH256) driver methods
 */

#include "ic_flash.h"

#ifdef USE_IC_FLASH
#ifndef USE_IC_SPI
	#define USE_IC_SPI
#endif
#include "ic_spi.h"
#ifndef USE_IC_WDT
	#define USE_IC_WDT
#endif
#include "ic_wdt.h"




/**
 * @fn flash_internal_page_erase ()
 * @brief erase page in internal flash
 * @param address of the first word in the page to be erased.
 */
void flash_internal_page_erase(uint32_t *page_address)
{
	/// Turn on flash erase enable and wait until the NVMC is ready
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	/// Erase page
	NRF_NVMC->ERASEPAGE = (uint32_t) page_address;

	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	/// Turn off flash erase enable and wait until the NVMC is ready
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
}

/**
 * @fn flash_internal_word_write ()
 * @brief function for filling a page in flash with a value
 * @param address of the first word in the page to be filled
 * @param value to be written to flash
 */
void flash_internal_word_write(uint32_t *address, uint32_t value){
	/// Turn on flash write enable and wait until the NVMC is ready:
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	*address = value;

	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	/// Turn off flash write enable and wait until the NVMC is ready:
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
}
void flash_internal_buf_write(uint32_t *address, uint32_t *value, uint32_t length){
	uint32_t i;
	/// Turn on flash write enable and wait until the NVMC is ready:
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

	for (i=0; i<length;++i){
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
		}
		*(address+i) = *(value+i);
	}
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	/// Turn off flash write enable and wait until the NVMC is ready:
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
}
void flash_internal_get_values(uint32_t *address, uint32_t *data, uint32_t length){
	uint32_t i;
	for (i=0; i<length;++i){
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
		}
		*(data+i) = *(address+i);
	}
}

///**
// * @fn flash_read_memory ()
// * @brief read data from flash memory
// * @param cell address
// * @param pointer to data buffer
// * @param length of data buffer
// */
//void flash_read_memory(uint32_t address, uint8_t* buffer, uint32_t buffer_len)
//{
//	uint8_t long_address = 0;
//
//	/// If 4 byte address, enter long address mode
//	if(address >= 0x1000000){
//		long_address = 1;
//		flash_enable_higher_bank();
//		address -= 0x1000000;
//	}
//
//	spi_master_cs_low(GPIO_FLASH_CS);
//
//	spi_write_byte(FLASH_COMMAND_READ_DATA);
//
//	spi_write_byte(address>>16);
//	spi_write_byte(address>>8);
//
//	spi_multipleRead_data(address, buffer, buffer_len);
//
//	spi_master_cs_high(GPIO_FLASH_CS);
//
//	if(long_address){
//		flash_disable_higher_bank();
//	}
//}
///**
// * @fn flash_erase_sector ()
// * @brief erase data sector
// * @param sector address
// */
//void flash_erase_sector(uint32_t address)
//{
//	uint8_t long_address = 0;
//
//	flash_wait_for_device_ready(FLASH_OPERATION_TIMEOUT);
//
//	/// if 4 byte address, enter long address mode
//	if(address >= 0x1000000){
//		long_address = 1;
//		flash_enable_higher_bank();
//		address -= 0x1000000;
//	}
//
//	flash_write_enable();
//
//	spi_master_cs_low(GPIO_FLASH_CS);
//
//	spi_write_byte(FLASH_COMMAND_ERASE_SECTOR);
//
//	spi_write_byte(address>>16);
//	spi_write_byte(address>>8);
//	spi_write_byte(address);
//
//	spi_master_cs_high(GPIO_FLASH_CS);
//
//	flash_wait_for_device_ready(FLASH_OPERATION_TIMEOUT);
//
//	flash_write_disable();
//
//	if(long_address){
//		flash_disable_higher_bank();
//	}
//}
///**
// * @fn flash_power_down ()
// * @brief set power down mode
// */
//void flash_power_down ()
//{
//	spi_master_cs_low(GPIO_FLASH_CS);
//	spi_write_byte(FLASH_COMMAND_POWER_DOWN);
//	spi_master_cs_high(GPIO_FLASH_CS);
//}
//
///**
// * @fn flash_power_up ()
// * @brief set power up mode
// */
//void flash_power_up ()
//{
//	spi_master_cs_low(GPIO_FLASH_CS);
//	spi_write_byte(FLASH_COMMAND_POWER_UP);
//	spi_master_cs_high(GPIO_FLASH_CS);
//}

#endif
