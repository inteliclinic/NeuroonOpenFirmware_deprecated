/*
 * ic_flash_enclosed.h
 *
 *  Created on: 27 paź 2015
 *      Author: kn
 */

/**
 * HACK:
 *
 * HOW to EPOCH MEASUREMENT:
 * you need to init module → iFLASH_ENCLOSED_init_after_restart(void) returns true;
 * than your module is ready to get data
 *
 * Single sleep version:
 * you need to init sleep measurement → FLASH_ENCLOSED_init_epoch_measurement returns true;
 * with 14 bytes of your data.
 * then you can add epochs by FLASH_ENCLOSED_write_epoch_measurement()
 * whith 18 bytes of your data.
 *
 * to read data you have → bool FLASH_ENCLOSED_get_sleep_frame returns true,
 *  if frame is present, otherwise returns false.
 *
 *  when above funs return false it could be done by lack of memory
 *  you can check it by  FLASH_ENCLOSED_get_free_space that returns count of available epochs
 *
 *  if you need info about how many sleep measurements is present in memory you should
 *  call FLASH_ENCLOSED_get_sleep_counter
 *
 *  if device is out of memory you can erease epoch space and begin to write from the begining
 *
 *  -------------------------------------------------------------------------------------------
 *
 *  HOW to RAW DATA MEASUREMENT:
 *	after restart module doesn't know if the raw data space is empty or not. It is user of this
 *	lib resposibility to check it.
 *
 *	WRITE:
 *	if you want to write raw data you need to call FLASH_ENCLOSED_raw_data_space_init.
 *	It is long, time consuming, blocking fun. 2 minutes could be taken.
 *	than it is simple:
 *	you need to add samples to fifo by:
 *										bool FLASH_ENCLOSED_add_raw_data_sample_to_fifo
 *										bool FLASH_ENCLOSED_add_raw_data_array_to_fifo
 *	and periodicaly call bool FLASH_ENCLOSED_write_raw_data, it latch data from fifo to FLASH if
 *	in FIFO is more than 256 samples.
 *	You have also fun to erase raw_data_spaca of flash → bool FLASH_ENCLOSED_erase_raw_data_space
 *
 *	READ:
 *	you need to call:
 *	FLASH_ENCLOSED_read_raw_data_init to set internal pointer to first frame of date
 *	than
 *	FLASH_ENCLOSED_read_next_raw_data gives you 256 bytes of data and returns true. When memory
 *	ends, fun returs false;
 *
 */

#ifndef IC_FLASH_ENCLOSED_H_
#define IC_FLASH_ENCLOSED_H_
#include "stdbool.h"
#include "ic_fifo.h"

#define FLASH_ENCLOSED_FIFO_BUFFER_SIZE 512

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

#define EXT_FLASH_DATA_FOR_EPOCHS_START			0x100000
#define EXT_FLASH_DATA_FOR_EPOCHS_END			0xFFFFFF
#define EXT_FLASH_DATA_FOR_EPOCHS_LENGTH		0xEFFFFF

//#define FLASH_OPERATION_TIMEOUT					1000

//FLASH MODULE to store your sleep history and raw data at the end.

//-----------------------INIT SECTION------------------------------
//Init function (only test if Flash works properly) @deprecated
bool FLASH_ENCLOSED_init(void);//done

//It is obligatory after restart to init FLASH module
bool FLASH_ENCLOSED_init_after_restart(void);//done

//void FLASH_ENCLOSED_deinit(void){return;}//done

//--------------- EPOCH MEASUREMENT FUNS ------------------------------

//returns how many 20 bytes cells are available for normal epochs
uint32_t FLASH_ENCLOSED_get_free_space(void);//done

//returns 20 byte frame from sleep_nr sleep, and frame_nr epoch to buf pointer
bool FLASH_ENCLOSED_get_sleep_frame(uint32_t sleep_nr, uint16_t frame_nr, uint8_t *buf);//done


//write 14 byte frame with init info from buf. have to be 14 byte length!
//it's your responsibility
//this fun creates header → bytes 0 and 1 : frame number → always 0x0000
//bytes 2 and 3 → sleep nr.
//bytes 4 and 5 → reserved
bool FLASH_ENCLOSED_init_epoch_measurement(uint8_t *buf);

//write 18 byte frame with measurements from epoch buf have to be at least 18 byte length!
//it's your responsibility
bool FLASH_ENCLOSED_write_epoch_measurement(uint8_t *buf);//done

//returns actual nr of sleep measurements
uint16_t FLASH_ENCLOSED_get_sleep_counter(void);

//Fun erase epoch space
bool FLASH_ENCLOSED_erase_epoch_space(void);

//--------------- RAW DATA FUNS ------------------------------
void FLASH_ENCLOSED_raw_data_space_init(void);
//if internal FIFO has more than 256 bytes, this function will write page to external Flash(256 bytes)
bool FLASH_ENCLOSED_write_raw_data(void);//done
// force to write page when fifo has less than 256 bytes to write
void FLASH_ENCLOSED_force_write_page(void);//done

void FLASH_ENCLOSED_raw_data_space_write_test(void);

//function to add sample(s) to internal Flash Fifo
synchronized bool FLASH_ENCLOSED_add_raw_data_sample_to_fifo(uint8_t sam);//done
synchronized bool FLASH_ENCLOSED_add_raw_data_array_to_fifo(uint8_t *buf, uint16_t len);//done

synchronized void FLASH_ENCLOSED_raw_data_clear_fifo(void);

//bool FLASH_ENCLOSED_is_higher_bank_enabled(void);

//function erase raw data space
bool FLASH_ENCLOSED_erase_raw_data_space(uint16_t x);//done

//read next page (256 bytes) from the beginnig of raw_data, return false when flash space ends
bool FLASH_ENCLOSED_read_next_raw_data(uint8_t *buf);//done
//function set the internal counter on first frame
void FLASH_ENCLOSED_read_raw_data_init(uint16_t start_address);//done



#endif /* IC_FLASH_ENCLOSED_H_ */
