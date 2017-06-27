/*
 * ic_flash_enclosed.c
 *
 *  Created on: 29 paź 2015
 *      Author: kn
 */

#include "ic_flash_enclosed.h"
#include "ic_lib_config.h"
#include <string.h>
///UART internal FIFO
static uint8_t buff[FLASH_ENCLOSED_FIFO_BUFFER_SIZE];
static FIFO_u8 flash_fifo__ = FIFO_INIT(buff, FLASH_ENCLOSED_FIFO_BUFFER_SIZE);
static FIFO_u8 *flash_fifo = &flash_fifo__;

//static uint32_t flash_read_adress=0;
//static bool flash_enabled_higher_bank=false;

struct FLASH_ENCLOSED_config_{
	uint8_t empty_parts;
	uint32_t raw_data_actual_adress;
	uint32_t epochs_data_actual_adress;
	bool flash_enabled_higher_bank;
	uint32_t flash_read_adress;
	uint32_t epochs_available;

	uint16_t sleep_counter;
	uint16_t frame_counter;

	uint32_t last_sleep_start_adress;
	uint32_t last_sleep_epochs_count;

	uint32_t searching_sleep_header_address;
	uint16_t searching_sleep_number;
	uint32_t searching_sleep_epochs_count;
};

volatile struct FLASH_ENCLOSED_config_ FLASH_ENCLOSED_config;

static void FLASH_ENCLOSED_write_command(uint8_t command);
static void FLASH_ENCLOSED_read_register(uint8_t addr, uint8_t* buffer);
static void FLASH_ENCLOSED_getID(uint8_t* buffer);
static bool FLASH_ENCLOSED_enable_higher_bank ();
static bool FLASH_ENCLOSED_disable_higher_bank();
static void FLASH_ENCLOSED_wait_for_device_ready(uint32_t timeout);
static void FLASH_ENCLOSED_write_enable ();
static void FLASH_ENCLOSED_write_disable ();
static void FLASH_ENCLOSED_read_memory_page_unsafe_256(uint32_t address, uint8_t* buffer);
static bool FLASH_ENCLOSED_erase_block(uint8_t address);
void FLASH_ENCLOSED_read_memory(uint32_t address, uint8_t* buffer, uint32_t buffer_len);

static void FLASH_ENCLOSED_write_data_to_single_page(uint32_t address, uint8_t* buffer, uint32_t buffer_len);
static void FLASH_ENCLOSED_write_data_continuously(uint32_t address, uint8_t* buffer, uint32_t buffer_len);
static uint8_t FLASH_ENCLOSED_is_empty (uint8_t mode);
static bool FLASH_ENCLOSED_epoch_bank_is_empty(void);
static uint32_t FLASH_ENCLOSED_find_last_sleep (void);
static uint32_t FLASH_ENCLOSED_find_last_epoch_addr(void);
static bool FLASH_ENCLOSED_find_sleep (uint16_t sleep_number);

void FLASH_ENCLOSED_erase_chip ()
{
	FLASH_ENCLOSED_wait_for_device_ready(FLASH_OPERATION_TIMEOUT);
	FLASH_ENCLOSED_write_enable();

	spi_master_cs_low(GPIO_FLASH_CS);
	spi_write_byte(FLASH_COMMAND_ERASE_CHIP);
	spi_master_cs_high(GPIO_FLASH_CS);

	FLASH_ENCLOSED_wait_for_device_ready(0xffffffff);
	FLASH_ENCLOSED_write_disable();
}
bool FLASH_ENCLOSED_erase_epoch_space(void){
	uint16_t i;
	while(!FLASH_ENCLOSED_disable_higher_bank());
	for(i=0x10;i<256;i++)
	{//
		if(!FLASH_ENCLOSED_erase_block((uint8_t)i))
				return false;

		WDT_RR();
		ic_delay_ms(10);
	}
	FLASH_ENCLOSED_config.epochs_data_actual_adress=EXT_FLASH_DATA_FOR_EPOCHS_START;
	FLASH_ENCLOSED_config.epochs_available=(0x1000000-EXT_FLASH_DATA_FOR_EPOCHS_START)/20;
	FLASH_ENCLOSED_config.sleep_counter=0;
	return true;

}
bool FLASH_ENCLOSED_erase_raw_data_space(uint16_t x){
	uint16_t i;
	if (x>256)
		x=256;
	while(!FLASH_ENCLOSED_enable_higher_bank());
	for(i=0;i<x;i++){
		if(!FLASH_ENCLOSED_erase_block((uint8_t)i))
				return false;

		WDT_RR();
		ic_delay_ms(10);
	}
	FLASH_ENCLOSED_config.raw_data_actual_adress=0x0;
	return true;
}
bool FLASH_ENCLOSED_init ()
{
	uint8_t check_frame[3];
	uint32_t check_value;

	spi_master_cs_init(GPIO_FLASH_CS);
	spi_master_cs_high(GPIO_FLASH_CS);

	/// check communication
	FLASH_ENCLOSED_getID(check_frame);
	check_value = ((uint32_t)(check_frame[0]))<<16 | ((uint32_t)(check_frame[1]))<<8 | (uint32_t)(check_frame[2]);

	FLASH_ENCLOSED_disable_higher_bank();

	if(check_value == FLASH_ID_VALUE)
		return true;
	else
		return false;
}
bool FLASH_ENCLOSED_init_after_restart ()
{
	uint32_t temp_adr;
	if(FLASH_ENCLOSED_init()==false)
		return false;

	WDT_RR();
	//init_module
	if(FLASH_ENCLOSED_epoch_bank_is_empty()){
		FLASH_ENCLOSED_config.epochs_data_actual_adress=EXT_FLASH_DATA_FOR_EPOCHS_START;
		FLASH_ENCLOSED_config.epochs_available=(0x1000000-EXT_FLASH_DATA_FOR_EPOCHS_START)/20;
		FLASH_ENCLOSED_config.searching_sleep_number=0xFFFF;
		FLASH_ENCLOSED_config.searching_sleep_header_address=0;
		FLASH_ENCLOSED_config.searching_sleep_epochs_count=0;
		FLASH_ENCLOSED_config.sleep_counter=0;
	}
	else{
		temp_adr = FLASH_ENCLOSED_find_last_epoch_addr();
		FLASH_ENCLOSED_config.epochs_available=(0x1000000-temp_adr)/20;
		FLASH_ENCLOSED_config.epochs_data_actual_adress=temp_adr;
		if (FLASH_ENCLOSED_find_last_sleep ()==0)
			return false;

		FLASH_ENCLOSED_config.searching_sleep_number=FLASH_ENCLOSED_config.sleep_counter;
		FLASH_ENCLOSED_config.searching_sleep_header_address=FLASH_ENCLOSED_config.last_sleep_start_adress;
		FLASH_ENCLOSED_config.searching_sleep_epochs_count=FLASH_ENCLOSED_config.last_sleep_epochs_count;

	}
	WDT_RR();

	FLASH_ENCLOSED_config.raw_data_actual_adress=0xFFFFFFFF;

	FLASH_ENCLOSED_config.flash_read_adress=0;
	return true;
}

uint32_t FLASH_ENCLOSED_get_free_space(){

	return FLASH_ENCLOSED_config.epochs_available;
}

bool FLASH_ENCLOSED_get_sleep_frame(uint32_t sleep_nr, uint16_t frame_nr, uint8_t *buf){
	uint8_t tmp[20];


	if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==true)
		while(!FLASH_ENCLOSED_disable_higher_bank());

	if (FLASH_ENCLOSED_config.searching_sleep_number!=sleep_nr){
		if(FLASH_ENCLOSED_find_sleep(sleep_nr)==false){
			buf[0]=0xff;
			buf[1]=0xff;
			buf[2]=0xff;
			buf[3]=0xff;
			buf[4]=0xff;
			return false;
		}
	}
	if(frame_nr==0){
		FLASH_ENCLOSED_read_memory(FLASH_ENCLOSED_config.searching_sleep_header_address, tmp,20);
		tmp[4]=(uint8_t)FLASH_ENCLOSED_config.searching_sleep_epochs_count;
		tmp[5]=(uint8_t)(FLASH_ENCLOSED_config.searching_sleep_epochs_count>>8);
		memcpy(buf, tmp, 20);
	}
	else if (frame_nr<FLASH_ENCLOSED_config.searching_sleep_epochs_count){
		FLASH_ENCLOSED_read_memory(FLASH_ENCLOSED_config.searching_sleep_header_address+(20*frame_nr), buf, 20);
	}
	else {
		buf[0]=0xff;
		buf[1]=0xff;
		buf[2]=0xff;
		buf[3]=0xff;
		buf[4]=0xff;
		return false;
	}
	return true;
}
bool FLASH_ENCLOSED_write_epoch_measurement(uint8_t *buf){
	uint8_t tmp[20];
	if(FLASH_ENCLOSED_get_free_space()==0)
		return false;

	if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==true)
		FLASH_ENCLOSED_disable_higher_bank();

	tmp[0]=(uint8_t)FLASH_ENCLOSED_config.frame_counter;
	tmp[1]=(uint8_t)(FLASH_ENCLOSED_config.frame_counter>>8);

	memcpy(tmp+2, buf, 18);

	FLASH_ENCLOSED_write_data_continuously(FLASH_ENCLOSED_config.epochs_data_actual_adress, tmp, 20);
	FLASH_ENCLOSED_config.epochs_data_actual_adress+=20;
	FLASH_ENCLOSED_config.epochs_available--;
	FLASH_ENCLOSED_config.frame_counter++;
	return true;
}
bool FLASH_ENCLOSED_init_epoch_measurement(uint8_t *buf){
	uint8_t tmp[18];
	FLASH_ENCLOSED_config.sleep_counter++;
	tmp[0]=(uint8_t)FLASH_ENCLOSED_config.sleep_counter;
	tmp[1]=(uint8_t)(FLASH_ENCLOSED_config.sleep_counter>>8);
	tmp[2]=0xff;
	tmp[3]=0xff;
	memcpy(tmp+4, buf, 14);

	FLASH_ENCLOSED_config.frame_counter=0;
	return FLASH_ENCLOSED_write_epoch_measurement(tmp);
}

uint16_t FLASH_ENCLOSED_get_sleep_counter(){
	return FLASH_ENCLOSED_config.sleep_counter;
}

void FLASH_ENCLOSED_raw_data_space_init(){

	FLASH_ENCLOSED_raw_data_clear_fifo();
	FLASH_ENCLOSED_config.raw_data_actual_adress=0;
}
bool FLASH_ENCLOSED_write_raw_data(void){
	uint8_t buf[256];
	uint16_t ind;

	if (FLASH_ENCLOSED_config.raw_data_actual_adress>=0xFFFFFF)
		return false;

	if(fifou8_getCount(flash_fifo)>=256){
		for(ind=0;ind<256;ind++){
			buf[ind]=fifou8_get(flash_fifo);
		}
		if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==false)
			FLASH_ENCLOSED_enable_higher_bank();

		FLASH_ENCLOSED_write_data_to_single_page(FLASH_ENCLOSED_config.raw_data_actual_adress, buf, 256);
		FLASH_ENCLOSED_config.raw_data_actual_adress+=256;
	}
	return true;
}

void FLASH_ENCLOSED_force_write_page(void){
	uint8_t buf[256];
	uint16_t ind=0,cnt1,cnt2;
	cnt2=cnt1=fifou8_getCount(flash_fifo);
	while(cnt1--){
		buf[ind++]=fifou8_get(flash_fifo);
	}

	if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==false)
			FLASH_ENCLOSED_enable_higher_bank();

	FLASH_ENCLOSED_write_data_to_single_page(FLASH_ENCLOSED_config.raw_data_actual_adress, buf, cnt2);
	FLASH_ENCLOSED_config.raw_data_actual_adress+=256;
}

void FLASH_ENCLOSED_raw_data_space_write_test(void){
	uint8_t buff[2];
	if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==false)
		while(!FLASH_ENCLOSED_enable_higher_bank());
	for (uint32_t i=0; i<=0xFFFF; ++i){
		buff[0]=(uint8_t)i;
		buff[1]=(uint8_t)(i>>8);
		FLASH_ENCLOSED_add_raw_data_array_to_fifo(buff, 2);
		FLASH_ENCLOSED_force_write_page();
	}
}

synchronized bool FLASH_ENCLOSED_add_raw_data_sample_to_fifo(uint8_t sam){
	return fifou8_sadd(flash_fifo, sam);
}
synchronized bool FLASH_ENCLOSED_add_raw_data_array_to_fifo(uint8_t *buf, uint16_t len){
	return fifou8_saddArray(flash_fifo, buf, len);
}

synchronized void FLASH_ENCLOSED_raw_data_clear_fifo(void){
	fifou8_sclear(flash_fifo);
}
bool FLASH_ENCLOSED_read_next_raw_data(uint8_t *buf){
	if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==false){
		FLASH_ENCLOSED_enable_higher_bank();
	}
	if (FLASH_ENCLOSED_config.flash_read_adress>0xFFFF00)
		return false;

	FLASH_ENCLOSED_read_memory_page_unsafe_256(FLASH_ENCLOSED_config.flash_read_adress, buf);
	FLASH_ENCLOSED_config.flash_read_adress+=256;

	return true;
}
void FLASH_ENCLOSED_read_raw_data_init(uint16_t start_address){
	FLASH_ENCLOSED_config.flash_read_adress=start_address<<8;
}

//=============================== STATIC FUNCTIONS DEFINITIONS =============================

static void FLASH_ENCLOSED_write_command(uint8_t command){
	spi_master_cs_low(GPIO_FLASH_CS);
	spi_write_byte(command);
	spi_master_cs_high(GPIO_FLASH_CS);
}
static void FLASH_ENCLOSED_read_register(uint8_t addr, uint8_t* buffer){
	spi_master_cs_low(GPIO_FLASH_CS);
	spi_read_data(addr, buffer);
	spi_master_cs_high(GPIO_FLASH_CS);
}
static void FLASH_ENCLOSED_getID(uint8_t* buffer)
{
	spi_master_cs_low(GPIO_FLASH_CS);
	spi_multipleRead_data(FLASH_COMMAND_READ_ID, buffer, 3);
	spi_master_cs_high(GPIO_FLASH_CS);
}

/**
 * @fn static FLASH_ENCLOSED_enable_higher_bank ()
 * @brief enable higher bank in flash memory
 */
static bool FLASH_ENCLOSED_enable_higher_bank ()
{
	uint8_t status;
	uint32_t counter = 0;

	FLASH_ENCLOSED_write_command(FLASH_COMMAND_HIGH_BANK_ENABLE);

	FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_INFO_REG, &status);
	while ((status & FLASH_HIGER_BANK_FLAG) == 0  && counter < FLASH_OPERATION_TIMEOUT) {
		counter++;
		FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_INFO_REG, &status);
	}
	if (counter>=FLASH_OPERATION_TIMEOUT){
		FLASH_ENCLOSED_config.flash_enabled_higher_bank=false;
		return false;
	}
	else{
		FLASH_ENCLOSED_config.flash_enabled_higher_bank=true;
		return true;
	}
}

/**
 * @fn static flash_disable_higher_bank ()
 * @brief disable higher bank in flash memory
 */
static bool FLASH_ENCLOSED_disable_higher_bank(){
	uint8_t status;
	uint32_t counter = 0;
	FLASH_ENCLOSED_write_command(0x98);

	//read information register
	FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_INFO_REG, &status);
	while ((status & FLASH_HIGER_BANK_FLAG) == 1  && counter < FLASH_OPERATION_TIMEOUT) {
		counter++;
		FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_INFO_REG, &status);
	}
	if (counter>=FLASH_OPERATION_TIMEOUT){
		FLASH_ENCLOSED_config.flash_enabled_higher_bank=true;
		return false;
	}
	else{
		FLASH_ENCLOSED_config.flash_enabled_higher_bank=false;
		return true;
	}
}
static void FLASH_ENCLOSED_wait_for_device_ready(uint32_t timeout)
{
	uint8_t status;
	uint32_t counter = 0;

	FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_STATUS_REG, &status);
	while ((status & FLASH_READY_FLAG) == 1 && counter < timeout) {
		counter++;
		FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_STATUS_REG, &status);
		WDT_RR();
	}
}
static void FLASH_ENCLOSED_write_enable ()
{
	uint8_t status;

	FLASH_ENCLOSED_write_command(FLASH_COMMAND_WRITE_ENABLE);

	FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_STATUS_REG, &status);
	while ((status & FLASH_WRITE_ENABLE_FLAG) == 0) {
		FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_STATUS_REG, &status);
	}
}
static void FLASH_ENCLOSED_write_disable ()
{
	uint8_t status;
	uint32_t counter;

	FLASH_ENCLOSED_write_command(FLASH_COMMAND_WRITE_DISABLE);

	FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_STATUS_REG, &status);
	while ((status & FLASH_WRITE_ENABLE_FLAG) == 1 && counter < FLASH_OPERATION_TIMEOUT) {
		counter++;
		FLASH_ENCLOSED_read_register(FLASH_COMMAND_READ_STATUS_REG, &status);
	}
}
static void FLASH_ENCLOSED_read_memory_page_unsafe_256(uint32_t address, uint8_t* buffer)
{
	spi_master_cs_low(GPIO_FLASH_CS);

	spi_write_byte(FLASH_COMMAND_READ_DATA);

	spi_write_byte(address>>16);
	spi_write_byte(address>>8);

	spi_multipleRead_data(address, buffer, 256);

	spi_master_cs_high(GPIO_FLASH_CS);
}
static bool FLASH_ENCLOSED_erase_block(uint8_t address)
{
	FLASH_ENCLOSED_wait_for_device_ready(FLASH_OPERATION_TIMEOUT);

	FLASH_ENCLOSED_write_enable();

	spi_master_cs_low(GPIO_FLASH_CS);

	spi_write_byte(FLASH_COMMAND_ERASE_BLOCK);

	spi_write_byte(address);
	spi_write_byte(0x00);
	spi_write_byte(0x00);

	spi_master_cs_high(GPIO_FLASH_CS);

	FLASH_ENCLOSED_wait_for_device_ready(FLASH_OPERATION_TIMEOUT);

	FLASH_ENCLOSED_write_disable();

	return true;
}
void FLASH_ENCLOSED_read_memory(uint32_t address, uint8_t* buffer, uint32_t buffer_len)
{
	spi_master_cs_low(GPIO_FLASH_CS);

	spi_write_byte(FLASH_COMMAND_READ_DATA);

	spi_write_byte(address>>16);
	spi_write_byte(address>>8);

	spi_multipleRead_data(address, buffer, buffer_len);

	spi_master_cs_high(GPIO_FLASH_CS);
}
static void FLASH_ENCLOSED_write_data_to_single_page(uint32_t address, uint8_t* buffer, uint32_t buffer_len)
{
        FLASH_ENCLOSED_write_enable();

	spi_master_cs_low(GPIO_FLASH_CS);
	spi_write_byte(FLASH_COMMAND_PAGE_PROGRAM);

	spi_write_byte(address>>16);
	spi_write_byte(address>>8);
	spi_multipleWrite_data(address, buffer, buffer_len);
	spi_master_cs_high(GPIO_FLASH_CS);

	FLASH_ENCLOSED_wait_for_device_ready(FLASH_OPERATION_TIMEOUT);
	FLASH_ENCLOSED_write_disable();
}

static void FLASH_ENCLOSED_write_data_continuously(uint32_t address, uint8_t* buffer, uint32_t buffer_len)
{
	/// check if buffer fits in single page
	if((address+buffer_len-1) <= (address | 0xFF)) {
		FLASH_ENCLOSED_write_data_to_single_page(address, buffer, buffer_len);
	}
	else {
		uint32_t first_page_length = (address | 0xFF) - address + 1;
		FLASH_ENCLOSED_write_data_to_single_page(address, buffer, first_page_length);
		uint32_t second_page_length = buffer_len - first_page_length;
		FLASH_ENCLOSED_write_data_to_single_page((address | 0xFF) + 1, buffer+first_page_length, second_page_length);
	}
}
//good fun for tests
/*
 * Mode 0 all block
 * Mode 1 part for backup data
 * Mode 2 part for sleep data (epochs)
 * Mode 3 part for raw data of last sleep
 *
*/
static uint8_t FLASH_ENCLOSED_is_empty (uint8_t mode)
{
	uint8_t tmp[32], i, result = 0xff;
	uint32_t addr;
	uint8_t ret_value=0;
	WDT_RR();
	if(mode==1||mode==0){
		for(addr = 0; addr < 0x100000; addr += 32){
			FLASH_ENCLOSED_read_memory(addr, tmp, 32);
			WDT_RR();
			for(i = 0; i < 32; i++){
				result &= tmp[i];
			}
			if(result != 0xff){
				ret_value|=1;
				break;
			}
		}
		result = 0xff;
	}
	WDT_RR();
	if(mode==2||mode==0){
		for(addr = 0x100000; addr < 0x1000000; addr += 32){
			FLASH_ENCLOSED_read_memory(addr, tmp, 32);
			WDT_RR();
			for(i = 0; i < 32; i++){
				result &= tmp[i];
			}
			if(result != 0xff){
				ret_value|=1<<1;
				break;
			}
			if(mode==2)
				break;
		}
		result = 0xff;
	}
	WDT_RR();
	if(mode==3||mode==0){
		FLASH_ENCLOSED_enable_higher_bank();
		for(addr = 0; addr < 0x1000000; addr += 32){
			FLASH_ENCLOSED_read_memory(addr, tmp, 32);
			WDT_RR();
			for(i = 0; i < 32; i++){
				result &= tmp[i];
			}
			if(result != 0xff){
				ret_value|=1<<2;
				break;
			}
		}
		FLASH_ENCLOSED_disable_higher_bank();
	}
	WDT_RR();
	return ret_value;
}

bool FLASH_ENCLOSED_epoch_bank_is_empty(){
	uint8_t tmp[32];
	uint8_t result=0xff;
	uint32_t i;

	FLASH_ENCLOSED_disable_higher_bank();

	FLASH_ENCLOSED_read_memory(0x100000, tmp, 32);
	WDT_RR();
	for(i = 0; i < 32; i++){
		result &= tmp[i];
	}
	return result==0xff;
}

/**
 * @fn find_last_sleep()
 * @brief find last sleep number in flash data
 */
static uint32_t FLASH_ENCLOSED_find_last_sleep ()
{
	uint8_t buf[20];
  if (FLASH_ENCLOSED_config.epochs_data_actual_adress == EXT_FLASH_DATA_BUFF_BASE)
	  return 0;
	else {
		uint32_t pack_size = 0;
		if(FLASH_ENCLOSED_config.flash_enabled_higher_bank==true){
			FLASH_ENCLOSED_disable_higher_bank();
		}
	  	FLASH_ENCLOSED_read_memory(FLASH_ENCLOSED_config.epochs_data_actual_adress-20, buf, 20);
		pack_size =(uint16_t) (buf[1]<<8) + buf[0]+1;

		FLASH_ENCLOSED_read_memory(FLASH_ENCLOSED_config.epochs_data_actual_adress - (pack_size)*20, buf, 20);
		if (buf[0]==0 && buf[1]==0)
			FLASH_ENCLOSED_config.sleep_counter = (uint16_t)((buf[3]<<8) + buf[2]);
		else
			return 0;

		FLASH_ENCLOSED_config.last_sleep_start_adress = FLASH_ENCLOSED_config.epochs_data_actual_adress - pack_size*20;
		FLASH_ENCLOSED_config.last_sleep_epochs_count = pack_size;
	}
  return FLASH_ENCLOSED_config.last_sleep_start_adress;
}
/** EXTERNAL FLASH FUNTIONS */

static uint32_t FLASH_ENCLOSED_find_last_epoch_addr ()
{

	uint32_t 	addr = EXT_FLASH_DATA_BUFF_BASE,
				length =EXT_FLASH_DATA_FOR_EPOCHS_LENGTH/20/2;
	uint16_t val_current;
	uint16_t val_prev;
	addr += 20*length;
	while(addr > EXT_FLASH_DATA_BUFF_BASE){
		FLASH_ENCLOSED_read_memory(addr, (uint8_t*)&val_current, 2);
		FLASH_ENCLOSED_read_memory(addr-20, (uint8_t*)&val_prev, 2);
		if(val_current == 0xffff && val_prev < 0xffff){
			return addr;
		} else if (val_current == 0xffff && val_prev == 0xffff){
			length /= 2;
			if(length == 0)
				length = 1;
			addr -= 20*length;
		}  else if (val_current < 0xffff){
			length /= 2;
			if(length == 0)
				length = 1;
			addr += 20*length;
		}
	}

	return EXT_FLASH_DATA_BUFF_BASE;
}


static bool FLASH_ENCLOSED_find_sleep (uint16_t sleep_number)
{
	uint16_t flash_sleep_number = 1000;
	uint32_t pack_size = 0;
	uint32_t temp_add1,temp_add2;
	uint8_t buf[20];

	if(sleep_number==0){
		return false;
	}
	if(FLASH_ENCLOSED_config.epochs_data_actual_adress==0x100000)
		return false;

	temp_add1=FLASH_ENCLOSED_config.epochs_data_actual_adress;

	while (flash_sleep_number > sleep_number)
	{
		FLASH_ENCLOSED_read_memory(temp_add1-20, buf, 20);
		pack_size = (uint16_t)(buf[1]<<8) + buf[0] +1;

		temp_add2 = temp_add1 - (pack_size)*20;

		if (temp_add2 < EXT_FLASH_DATA_BUFF_BASE){
			return false;
		}
		else {
			temp_add1=temp_add2;

			FLASH_ENCLOSED_read_memory(temp_add1, buf, 20);
			if (buf[0]==0x00 && buf[1]==0)
				flash_sleep_number = (uint16_t)((buf[3]<<8) + buf[2]);
			else
				return false;
		}
	}

	if(flash_sleep_number==sleep_number){
		FLASH_ENCLOSED_config.searching_sleep_epochs_count=pack_size;
		FLASH_ENCLOSED_config.searching_sleep_header_address=temp_add1;
		FLASH_ENCLOSED_config.searching_sleep_number=flash_sleep_number;
		return true;
	}

	return false;
}


void for_test(void)
{
	uart_send_u8(FLASH_ENCLOSED_is_empty(0));
	{
		uint32_t last_sleep_num=FLASH_ENCLOSED_find_last_sleep();
		uart_send_u8(last_sleep_num>>24);
		uart_send_u8(last_sleep_num>>16);
		uart_send_u8(last_sleep_num>>8);
		uart_send_u8(last_sleep_num);
	}
}

