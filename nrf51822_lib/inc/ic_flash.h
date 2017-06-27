#ifndef __FLASH_DRIVER__H
#define __FLASH_DRIVER__H

#include "ic_lib_config.h"

/** COMMANDS */
#define FLASH_COMMAND_WRITE_ENABLE				0x06	/// Enable writing mode
#define FLASH_COMMAND_WRITE_DISABLE				0x04	/// Disable writing mode
#define FLASH_COMMAND_READ_STATUS_REG			0x05	/// Read Status register
#define FLASH_COMMAND_HIGH_BANK_ENABLE			0x67	/// Use address higher than 0xFFFFFF
#define FLASH_COMMAND_HIGH_BANK_DISABLE			0x98 	/// Disable use address higher than 0xFFFFFF
#define FLASH_COMMAND_READ_INFO_REG				0x2B	/// Read information register

#define FLASH_COMMAND_PAGE_PROGRAM				0x02	/// Start programming
#define FLASH_COMMAND_ERASE_SECTOR				0x20	/// Erase sector
#define FLASH_COMMAND_ERASE_BLOCK				0xD8	/// Erase block (512kB)
#define FLASH_COMMAND_ERASE_CHIP				0x60	/// Erase chip

#define FLASH_COMMAND_READ_MAN_ID				0x90	/// Manufacturer Device ID
#define FLASH_COMMAND_READ_ID					0x9F	/// ID

#define FLASH_COMMAND_READ_DATA					0x03	/// Read data form device

#define FLASH_COMMAND_POWER_DOWN				0xB9	/// Set power down mode
#define FLASH_COMMAND_POWER_UP					0xAB	/// Set power up mode

/** REGISTER VALUES */
#define FLASH_READY_FLAG						0x01
#define FLASH_WRITE_ENABLE_FLAG					0x02
#define FLASH_HIGER_BANK_FLAG					0x80
#define FLASH_ID_VALUE							0x1C7019	/// Manufacturer ID value

/** FUNCTIONS */

/** INTERNAL FLASH FUNCTIONS */
void flash_internal_page_erase(uint32_t *page_address);
void flash_internal_word_write(uint32_t *address, uint32_t value);
void flash_internal_buf_write(uint32_t *address, uint32_t *value, uint32_t length);
void flash_internal_get_values(uint32_t *address, uint32_t *data, uint32_t length);

/** EXTERNAL FLASH READ/WRITE FUNCTIONS */
void flash_read_register(uint8_t addr, uint8_t* buffer);
void flash_read_memory(uint32_t address, uint8_t* buffer, uint32_t buffer_len);
void flash_write_memory(uint32_t address, uint8_t* buffer, uint32_t buffer_len);

/** EXTERNAL FLASH ERASE FUNCTIONS */
void flash_erase_sector(uint32_t address);
void flash_erase_block(uint32_t address);
void flash_erase_chip(void);

/** EXTERNAL FLASH APPLICATION FUNCTIONS */
bool flash_init(void);
void flash_power_down(void);
void flash_power_up(void);
uint8_t flash_is_empty(void);

#endif
