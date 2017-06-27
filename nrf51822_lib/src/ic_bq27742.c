
/**
 * @file
 * @brief BQ27742 driver methods
 */

#include "ic_bq27742.h"
#include "ic_ltc3220.h"
#include "ic_delay.h"
#include "ic_i2c.h"
#include <math.h>
#include "ic_cli.h"
#include <limits.h>

const float BQ27742_CC_GAIN=4.768/54.931;
const float BQ27742_CC_DELTA=5677445.0/54.810;

/*#define FORCE_ENDIANNESS __ORDER_LITTLE_ENDIAN__*/
#define FORCE_ENDIANNESS __ORDER_BIG_ENDIAN__

#define SWAP_2BYTES(val) (\
    (((*(typeof(val) *)&(val)) >> CHAR_BIT) & 0x00FF) | \
    (((*(typeof(val) *)&(val)) << CHAR_BIT) & 0xFF00) )

#define SWAP_4BYTES(val) (\
    (((*(typeof(val) *)&(val)) >> CHAR_BIT*3) & 0x000000FF) | \
    (((*(typeof(val) *)&(val)) >> CHAR_BIT)   & 0x0000FF00) | \
    (((*(typeof(val) *)&(val)) << CHAR_BIT)   & 0x00FF0000) | \
    (((*(typeof(val) *)&(val)) << CHAR_BIT*3) & 0xFF000000) )

#if FORCE_ENDIANNESS == __BYTE_ORDER__
  #define ENSURE_ENDIANNESS(T) _Generic((T), \
      float: *(uint32_t *)&T,\
      default: T\
      )
#else
  #define ENSURE_ENDIANNESS(T) _Generic((T), \
      uint8_t: T,\
      int8_t: T,\
      uint16_t: SWAP_2BYTES(T),\
      uint32_t: SWAP_4BYTES(T),\
      int16_t: SWAP_2BYTES(T),\
      int32_t: SWAP_4BYTES(T),\
      float: SWAP_4BYTES(T),\
      default: 0\
      )
#endif

#define bq_subclass(structure_name) union{\
  struct structure_name data_structure;\
  uint8_t raw[sizeof(struct structure_name)];\
}bq_subclass_ ## structure_name;

float fast_power(float base, int16_t index){
  if (index<0)
    return fast_power(base, ++index) * 1/base;
  return index == 0 ? 1 : fast_power(base, --index) * base;
}

uint32_t float_to_bq_format (float val)
{
  const float _2pow_25 = 2.98023224e-8;
  union {
    uint32_t raw;
    uint8_t  byte[4];
  } ret_val;

  int16_t exp=0;
  float mod_val;
  float tmp_val;

  mod_val=fabsf(val);

  tmp_val=mod_val*(1.0 + _2pow_25);

  if(tmp_val <0.5)
  {
    do
    {
      tmp_val*=2;
      exp--;
    } while(tmp_val<0.5);
  }
  else if(tmp_val>=1.0)
  {
    do
    {
      tmp_val/=2;
      exp++;
    } while(tmp_val>=1.0);
  }

  if(exp>127)
    exp=127;
  else if(exp<-128)
    exp=-128;

  tmp_val=(fast_power(2.0,(8-exp))*mod_val)-128.0;

  ret_val.byte[3]=exp+128;
  ret_val.byte[2]=(uint8_t)tmp_val;
  tmp_val=256*(tmp_val-ret_val.byte[2]);
  ret_val.byte[1]=(uint8_t)tmp_val;
  tmp_val=256*(tmp_val-ret_val.byte[1]);
  ret_val.byte[0]=(uint8_t)tmp_val;

  if(val<0)
    ret_val.byte[2] |= 0x80;

  return ret_val.raw;
}

void bq27742_program_flash_subclass_data_104()
{
  uint8_t subclass_id=104;
  uint8_t flash_block=0;
  uint8_t offset=0;

  struct __attribute__((packed)) data_104{
    uint32_t cc_gain;
    uint32_t cc_delta;
    uint16_t cc_offset;
    uint8_t board_offset;
    uint8_t int_temp_offset;
    uint8_t ext_temp_offset;
    uint8_t pack_v_offset;
  };

  bq_subclass(data_104);

  bq_subclass_data_104.data_structure.cc_gain         = float_to_bq_format(BQ27742_CC_GAIN);
  bq_subclass_data_104.data_structure.cc_delta        = float_to_bq_format(BQ27742_CC_DELTA);
  bq_subclass_data_104.data_structure.cc_offset       = BQ27742_CC_OFFSET_;
  bq_subclass_data_104.data_structure.board_offset    = BQ27742_BOARD_OFFSET_;
  bq_subclass_data_104.data_structure.int_temp_offset = BQ27742_INT_TEMP_OFFSET;
  bq_subclass_data_104.data_structure.ext_temp_offset = BQ27742_EXT_TEMP_OFFSET;
  bq_subclass_data_104.data_structure.pack_v_offset   = BQ27742_PACK_V_OFFSET;

  bq_subclass_data_104.data_structure.cc_gain         = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.cc_gain);
  bq_subclass_data_104.data_structure.cc_delta        = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.cc_delta);
  bq_subclass_data_104.data_structure.cc_offset       = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.cc_offset);
  bq_subclass_data_104.data_structure.board_offset    = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.board_offset);
  bq_subclass_data_104.data_structure.int_temp_offset = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.int_temp_offset);
  bq_subclass_data_104.data_structure.ext_temp_offset = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.ext_temp_offset);
  bq_subclass_data_104.data_structure.pack_v_offset   = ENSURE_ENDIANNESS(bq_subclass_data_104.data_structure.pack_v_offset);

  uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
  uint32_t flash_date_sum = 0;
  uint8_t checksum = 0;
  uint8_t flash_date_access = 0x00;
  uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_data_104.raw, sizeof(bq_subclass_data_104));
  TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
  WDT_RR();
  for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
    flash_date_sum +=reg_value[i];

  checksum = 255 - (uint8_t)(flash_date_sum);
  print_cli ("checksum: %d\n\r", checksum);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
  ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
  WDT_RR();
}


/*************************WRTIE/READ FUNCTION*************************************/
/**
 * @fn bq27742_read_control_data (uint16_t subcommand)
 * @brief read data from  control register
 *  param[in] subcommand: data subcommand code
 *  param[out] read control data
 */
uint16_t bq27742_read_control_data (uint16_t subcommand)
{
	uint16_t return_value =0;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, (uint8_t*) &subcommand, sizeof(subcommand));
	ic_delay_ms(1);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, (uint8_t*) &return_value, sizeof(return_value));

	return return_value;
}

/**
 * @fn bq27742_write_control_data (uint8_t subcommand, uint8_t *data)
 * @brief write data to control register
 *  param[in] subcommand: data subcommand code
 */
void bq27742_write_control_data (uint16_t subcommand)
{
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, (uint8_t*) &subcommand, sizeof(subcommand));
}

/**
 * @fn bq27742_read_reg_data  (uint8_t command)
 * @brief read data from  general register
 *  param[in] command: data command code
 *  param[out] read register data
 */
uint16_t bq27742_read_reg_data (uint8_t command)
{
	uint16_t return_value =0;

	TWI_ReadReg (TWI_BQ27742_ADDRESS, command, (uint8_t*) &return_value, sizeof(return_value));

	return return_value;
}

/**
 * @fn bq27742_write_reg_data  (uint8_t command, uint16_t value)
 * @brief read data from  general register
 *  param[in] command: data command code
 *  param[in] value to write
 */
void bq27742_write_reg_data (uint8_t command, uint16_t value)
{
	TWI_WriteReg (TWI_BQ27742_ADDRESS, command, (uint8_t*) &value, sizeof(value));
}

/**
 * @fn bq27742_data_flash_write(uint8_t subclass_id, uint8_t flash_block, uint8_t offset, uint16_t value)
 * @brief write data to the data flash
 *  param[in] subclass_id: id number of configuration class
 *  param[in] flash_block: number of page in flash block (block more than 32B)
 *  param[in] offset: index into the BlockDate() command space
 *  param[in] value: data to write
 *  param[in] reg size: size of register (1 - 1B; 2 - 2B) depends on the size of value to write
 */
void bq27742_data_flash_write(uint8_t subclass_id, uint8_t flash_block, uint8_t offset, uint16_t value, uint8_t reg_size)
{
        if (reg_size == 2)
          value = ENSURE_ENDIANNESS(value);

	uint8_t i = 0;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;

	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);

	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, (uint8_t *)&value, reg_size);

	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, BQ27742_DATA_FLASH_BLOCK_SIZE);
	for (i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
}


/**
 * @fn bq27742_data_flash_read(uint8_t subclass_id, uint8_t flash_block, uint8_t offset)
 * @brief read data from the data flash
 *  param[in] subclass_id: id number of configuration class
 *  param[in] flash_block: number of page in flash block (block more than 32B) (wartośc bezpośrednio z bqStudio)
 *  flash_block to offset z reference manuala modulo 32
 *  param[in] offset: index into the BlockDate() command space
 *			offset to przesunięcie w danym 32 bajtowym bolku jaki dostaniemy
 *  param[out] value of read data
 *
 *  	przyklad rejestr "Term V Delta" ma nastepujące dane:
 *			id=80, flash_block=2 (66mod32) flash_block=2 reg_size=2 (bo to int 16)
 */
uint16_t bq27742_data_flash_read(uint8_t subclass_id, uint8_t flash_block, uint8_t offset, uint8_t reg_size)
{
	uint16_t read_value = 0;

	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);

	TWI_ReadReg (TWI_BQ27742_ADDRESS, block_date_address, (uint8_t *)&read_value, reg_size);

	ic_delay_ms(BQ27742_REGISTER_READ_DELAY);
	return read_value;
}

/**
 * @fn bq27742_control_status_read ()
 * @brief read data from control status register
 *  param[out] register value
 */
uint16_t bq27742_control_status_read ()
{
	uint16_t check_value = 0;

	check_value = bq27742_read_control_data (BQ27742_CONTROL_STATUS);

	return check_value;
}

/**
 * @fn bq27742_flags_read ()
 * @brief read data from flags register
 *  param[out] register value
 */
uint16_t bq27742_flags_read ()
{
	uint16_t flag_value =0;

	flag_value = bq27742_read_reg_data(BQ27742_FLAGS);

	return flag_value;
}

/**
 * @fn bq27742_safety_status_read ()
 * @brief read data from safety status register
 *  param[out] register value
 */
uint8_t bq27742_safety_status_read ()
{
	uint8_t safety_value =0;

	safety_value = (uint8_t) (bq27742_read_reg_data(BQ27742_SAFETY_STATUS));

	return safety_value;
}

/**
 * @fn bq27742_protector_status_read ()
 * @brief read data from protector status register
 *  param[out] register value
 */
uint8_t bq27742_protector_status_read ()
{
	uint8_t safety_value =0;

	safety_value = (uint8_t)(bq27742_read_reg_data(BQ27742_PROTECT_STATUS));

	return safety_value;
}

/*************************CONFIGURATION FUNCTIONS*************************************/

/**
 * @fn bq27742_check_communication ()
 * @brief check communication with device by read device type register (0x0742)
 *  param[out] return true if establish communication
 */
bool bq27742_check_communication ()
{
	uint16_t check_value =0;

	check_value = bq27742_read_control_data (BQ27742_DEVICE_TYPE);

	if (check_value == BQ27742_DEVICE_TYPE_VALUE)
		return true;
	else
		return false;
}

/**
 * @fn bq27742_sealed_set ()
 * @brief put device into sealed state
 *  param[out] return true if device enter sealed state
 */
bool bq27742_sealed_set ()
{
	uint16_t check_value = 0;
	uint16_t sealed_value = 1<<BQ_FULL_SEALED_MODE | 1<<BQ_SEALED_MODE;

	bq27742_write_control_data(BQ27742_SEALED);
	ic_delay_ms(1);

	check_value = bq27742_control_status_read();

	if (check_value && sealed_value)
		return true;
	else
		return false;
}

/**
 * @fn  bq27742_unsaled_set ()
 * @brief put device into unsealed state
 *  param[out] return true if device enter unsealed state
 */
bool bq27742_unsaled_set ()
{
	uint16_t check_value = 0;
	uint16_t sealed_value = 1<<BQ_FULL_SEALED_MODE;
	uint8_t key1 [2] = {0x14,0x04};
	uint8_t key2 [2] = {0x72,0x36};
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, key1, 2);
	ic_delay_ms(1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, key2, 2);
	ic_delay_ms(1);

	check_value = bq27742_control_status_read();
	if (check_value & sealed_value)
		return false;
	else
		return true;
}

/**
 * @fn  bq27742_full_access_set ()
 * @brief put device into full access state
 *  param[out] return true if device enter full access state
*/
bool bq27742_full_access_set ()
{
	uint16_t check_value = 0;
	uint16_t sealed_value = 1<<BQ_SEALED_MODE;
	uint8_t key1 [2] = {0xFF,0xFF};
	uint8_t key2 [2] = {0xFF,0xFF};
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, key1, 2);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_CONTROL, key2, 2);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	check_value = bq27742_control_status_read();
	if (check_value & sealed_value)
		return false;
	else
		return true;
}

/*************************APPLICATION FUNCTIONS*************************************/
/**
 * @fn  bq27742_init ()
 * @brief initialization device by set full access state
 *  param[out] return true if device init
*/
bool bq27742_init ()
{
	bool error_check = false;

	error_check = bq27742_check_communication ();

	return error_check;
}

/**
 * @fn bq27742_reset ()
 * @brief full reset device
 */
void bq27742_reset ()
{
	bq27742_write_control_data(BQ27742_RESET);
}

/**
 * @fn bq27742_shutdown ()
 * @brief shut down the device
 */
void bq27742_shutdown ()
{
	bq27742_write_control_data(BQ27742_SET_SHUTDOWN);
}

/**
 * @fn  bq27742_read_measurement_data (uint8_t *data)
 * @brief read measurement data form device
 *  param[in] data: pointer to the variable when data are read
*/
void bq27742_read_measurement_data (uint8_t *data)
{
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_TEMPERATURE, &data[0], 2);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_VOLTAGE, &data[2], 2);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_AVERAGE_CURRENT, &data[4], 2);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_STATE_OF_CHARGE, &data[6], 2);
}

int16_t bq27742_read_measurement_current_data ()
{
	int16_t data;
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_AVERAGE_CURRENT, (uint8_t *)&data, sizeof(data));
	return data;
}

uint16_t bq27742_soc_read ()
{
	return bq27742_read_reg_data (BQ27742_STATE_OF_CHARGE);
}

uint16_t bq27742_voltage_read ()
{
	return bq27742_read_reg_data (BQ27742_VOLTAGE);
}

bool bq27742_battery_full_charged ()
{
	uint16_t flag_value = 0;

	flag_value = bq27742_flags_read ();
	if(flag_value & (1<<FC))
		return true;
	else
		return false;
}

bool bq27742_battery_soc1_threshold ()
{
	uint16_t flag_value = 0;

	flag_value = bq27742_flags_read ();
	if(flag_value & (1<<SOC1))
		return true;
	else
		return false;
}

bool bq27742_battery_socf_threshold ()
{
	uint16_t flag_value = 0;

	flag_value = bq27742_flags_read ();
	if(flag_value & (1<<SOCF))
		return true;
	else
		return false;
}

bool bq27742_battery_high_detected ()
{
	uint16_t flag_value = 0;

	flag_value = bq27742_flags_read ();
	if(flag_value & (1<<BATHI))
		return true;
	else
		return false;
}

bool bq27742_battery_low_detected ()
{
	uint16_t flag_value = 0;

	flag_value = bq27742_flags_read ();
	if(flag_value & (1<<BATLOW))
		return true;
	else
		return false;
}

bool bq27742_charging_error_detected ()
{
	uint16_t safety_status = 0;
	uint8_t charging_error_mask = 1 << OVP_S | 1 << OTC_S | 1 << TDD_S | 1 << ISD_S;
	safety_status = bq27742_safety_status_read();
	if(safety_status & charging_error_mask)
		return true;
	else
		return false;
}

bool bq27742_discharging_error_detected ()
{
	uint16_t safety_status = 0;
	uint8_t charging_error_mask = 1 << UVP_S | 1 << OTD_S | 1 << TDD_S | 1 << ISD_S;
	safety_status = bq27742_safety_status_read();
	if(safety_status & charging_error_mask)
		return true;
	else
		return false;
}

/** CHG_FET off */
bool bq27742_charge_disable()
{
	uint16_t reg_value = 0;
	uint16_t command = 0x40 | BQ27742_CHG_DIS;

	bq27742_write_reg_data(BQ27742_FET_TEST, command);
	bq27742_write_control_data(BQ27742_START_FET_TEST);

	reg_value = bq27742_read_reg_data (BQ27742_FET_TEST);
	if (reg_value & BQ27742_CHG_DIS)
		return true;
	else
		return false;
}

/** DSG_FET off */
bool bq27742_discharge_disable()
{
	uint16_t reg_value = 0;
	uint16_t command = 0x40 | BQ27742_DSG_DIS;

	bq27742_write_reg_data(BQ27742_FET_TEST, command);
	bq27742_write_control_data(BQ27742_START_FET_TEST);

	reg_value = bq27742_read_reg_data (BQ27742_FET_TEST);
	if (reg_value & BQ27742_DSG_DIS)
		return false;
	else
		return true;
}

/** CHG_FET and DSG_FET on */
bool bq27742_charge_discharge_enable()
{
	uint16_t reg_value = 0;
	uint16_t command = 0x44;

	bq27742_write_reg_data(BQ27742_FET_TEST, command);
	bq27742_write_control_data(BQ27742_START_FET_TEST);

	reg_value = bq27742_read_reg_data (BQ27742_FET_TEST);
	if (reg_value & (BQ27742_CHG_DIS | BQ27742_DSG_DIS))
		return false;
	else
		return true;
}
/** BATTERY STATE MEASUREMENT */

/// TODO przerobic na czytanie stanu z BQ
/**
 * @fn battery_measurement ()
 * @brief check battery state of charge
 * @return battery level
 */
uint8_t bq_battery_measurement ()
{
	uint16_t rem_cap=bq27742_read_reg_data(BQ27742_REMAINING_CAPACITY);
	uint16_t full_chrg_cap=bq27742_read_reg_data(BQ27742_FULL_CHARGE_CAPACITY);
        /*
	 *uint16_t nom_available_cap=bq27742_read_reg_data(BQ27742_NOM_AVAILABLE_CAPACITY);
	 *uint16_t full_available_cap=bq27742_read_reg_data(BQ27742_FULL_AVAILABLE_CAPACITY);
         *print_cli(
         *    "\n\rRemaining cap: %d\n\rFull charge cap: %d\n\r"\
         *    "Nominal available cap: %d\n\rFull available cap: %d\n\r",
         *    rem_cap, full_chrg_cap, nom_available_cap, full_available_cap);
         */
	return (uint8_t)(((float)rem_cap/full_chrg_cap)*100.0f);
}

uint8_t bq_battery_cap_params(uint16_t *params){
  params[0]=bq27742_read_reg_data(BQ27742_REMAINING_CAPACITY);
  params[1]=bq27742_read_reg_data(BQ27742_FULL_CHARGE_CAPACITY);
  params[2]=bq27742_read_reg_data(BQ27742_NOM_AVAILABLE_CAPACITY);
  params[3]=bq27742_read_reg_data(BQ27742_FULL_AVAILABLE_CAPACITY);
  return 0;
}

/**
 * @fn bq_is_charging_allowed()
 * @brief check if charging is allowed by BQ
 * @return is charging allowed info
 */
bool bq_is_charging_allowed()
{
	uint16_t flags = bq27742_flags_read ();
	bool is_charging_allowed = (flags >> 3) & 1;
	return is_charging_allowed;
}

// procedury do programowania
void bq27742_program_flash_subclass_safety(){
  uint8_t subclass_id=BQ27742_SAFETY_CLASS_ID;
  uint8_t flash_block=0;
  uint8_t offset=0;

  struct __attribute__((packed)) safety{
    uint16_t  ov_prot_trshold;
    uint8_t   ov_prot_delay;
    uint16_t  ov_prot_recovery;
    uint16_t  uv_prot_threshold;
    uint8_t   uv_prot_delay;
    uint16_t  uv_prot_recovery;
    uint16_t  body_diode_threshold;
    uint16_t  ot_chg;
    uint8_t   ot_chg_time;
    uint16_t  ot_chg_recovery;
    uint16_t  ot_dsg;
    uint8_t   ot_dsg_time;
    uint16_t  ot_dsg_recovery;
  };

  bq_subclass(safety);

  bq_subclass_safety.data_structure.ov_prot_trshold       = BQ27742_OV_PROT_THRESHOLD;
  bq_subclass_safety.data_structure.ov_prot_delay         = BQ27742_OV_PROT_DELAY;
  bq_subclass_safety.data_structure.ov_prot_recovery      = BQ27742_OV_PROT_RECOVERY;
  bq_subclass_safety.data_structure.uv_prot_threshold     = BQ27742_UV_PROT_THRESHOLD;
  bq_subclass_safety.data_structure.uv_prot_delay         = BQ27742_UV_PROT_DELAY;
  bq_subclass_safety.data_structure.uv_prot_recovery      = BQ27742_UV_PROT_RECOVERY;
  bq_subclass_safety.data_structure.body_diode_threshold  = BQ27742_BODY_DIODE_THRESHOLD;
  bq_subclass_safety.data_structure.ot_chg                = BQ27742_OT_CHG;
  bq_subclass_safety.data_structure.ot_chg_time           = BQ27742_OT_CHG_TIME;
  bq_subclass_safety.data_structure.ot_chg_recovery       = BQ27742_OT_CHG_RECOVERY;
  bq_subclass_safety.data_structure.ot_dsg                = BQ27742_OT_DSG;
  bq_subclass_safety.data_structure.ot_dsg_time           = BQ27742_OT_DSG_TIME;
  bq_subclass_safety.data_structure.ot_dsg_recovery       = BQ27742_OT_DSG_RECOVERY;

  bq_subclass_safety.data_structure.ov_prot_trshold       = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ov_prot_trshold);
  bq_subclass_safety.data_structure.ov_prot_delay         = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ov_prot_delay);
  bq_subclass_safety.data_structure.ov_prot_recovery      = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ov_prot_recovery);
  bq_subclass_safety.data_structure.uv_prot_threshold     = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.uv_prot_threshold);
  bq_subclass_safety.data_structure.uv_prot_delay         = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.uv_prot_delay);
  bq_subclass_safety.data_structure.uv_prot_recovery      = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.uv_prot_recovery);
  bq_subclass_safety.data_structure.body_diode_threshold  = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.body_diode_threshold);
  bq_subclass_safety.data_structure.ot_chg                = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ot_chg);
  bq_subclass_safety.data_structure.ot_chg_time           = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ot_chg_time);
  bq_subclass_safety.data_structure.ot_chg_recovery       = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ot_chg_recovery);
  bq_subclass_safety.data_structure.ot_dsg                = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ot_dsg);
  bq_subclass_safety.data_structure.ot_dsg_time           = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ot_dsg_time);
  bq_subclass_safety.data_structure.ot_dsg_recovery       = ENSURE_ENDIANNESS(bq_subclass_safety.data_structure.ot_dsg_recovery);

  uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
  uint32_t flash_date_sum = 0;
  uint8_t checksum = 0;
  uint8_t flash_date_access = 0x00;
  uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_safety.raw, sizeof(bq_subclass_safety));
  TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
  WDT_RR();
  for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
    flash_date_sum +=reg_value[i];

  checksum = 255 - (uint8_t)(flash_date_sum);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
  ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
  WDT_RR();
}

void bq27742_program_flash_subclass_charge()
{
	uint8_t subclass_id=BQ27742_CHARGE_CLASS_ID;

	struct __attribute__((packed)) charge{
		uint16_t charging_voltage;
	};

	bq_subclass(charge);

	bq_subclass_charge.data_structure.charging_voltage         = BQ27742_CHARGING_VOLTAGE_;

	bq_subclass_charge.data_structure.charging_voltage         = ENSURE_ENDIANNESS(bq_subclass_charge.data_structure.charging_voltage);

	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;
		
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_charge.raw, sizeof(bq_subclass_charge));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_charge_termination()
{
	uint8_t subclass_id=36;

	struct __attribute__((packed)) charge_termination{
	    uint16_t  taper_current;
	    uint16_t  min_taper_capacity;
	    uint16_t  taper_voltage;
	    uint8_t   current_taper_window;
	    uint8_t   tca_set;
	    uint8_t   tca_clear_procent;
	    uint8_t   fc_set_procent;
	    uint8_t   fc_clear_procent;
	    uint16_t  dodateoc_delta_t;
	};

	bq_subclass(charge_termination);

	bq_subclass_charge_termination.data_structure.taper_current         = BQ27742_TAPER_CURRENT;
	bq_subclass_charge_termination.data_structure.min_taper_capacity    = BQ27742_MIN_TAPER_CAPACITY;
	bq_subclass_charge_termination.data_structure.taper_voltage         = BQ27742_TAPER_VOLTAGE;
	bq_subclass_charge_termination.data_structure.current_taper_window  = BQ27742_CURRENT_TAPER_WINDOW;
	bq_subclass_charge_termination.data_structure.tca_set               = BQ27742_TCA_SET;
	bq_subclass_charge_termination.data_structure.tca_clear_procent     = BQ27742_TCA_CLEAR_PROCENT;
	bq_subclass_charge_termination.data_structure.fc_set_procent        = BQ27742_FC_SET_PROCENT;
	bq_subclass_charge_termination.data_structure.fc_clear_procent      = BQ27742_FC_CLEAR_PROCENT;
	bq_subclass_charge_termination.data_structure.dodateoc_delta_t      = BQ27742_DODATEOC_DELTA_T;

	bq_subclass_charge_termination.data_structure.taper_current         = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.taper_current);
	bq_subclass_charge_termination.data_structure.min_taper_capacity    = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.min_taper_capacity);
	bq_subclass_charge_termination.data_structure.taper_voltage         = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.taper_voltage);
	bq_subclass_charge_termination.data_structure.current_taper_window  = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.current_taper_window);
	bq_subclass_charge_termination.data_structure.tca_set               = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.tca_set);
	bq_subclass_charge_termination.data_structure.tca_clear_procent     = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.tca_clear_procent);
	bq_subclass_charge_termination.data_structure.fc_set_procent        = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.fc_set_procent);
	bq_subclass_charge_termination.data_structure.fc_clear_procent      = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.fc_clear_procent);
	bq_subclass_charge_termination.data_structure.dodateoc_delta_t      = ENSURE_ENDIANNESS(bq_subclass_charge_termination.data_structure.dodateoc_delta_t);

	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_charge_termination.raw, sizeof(bq_subclass_charge_termination));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_JEITA()
{
	uint8_t subclass_id=39;

	struct __attribute__((packed)) jeita{
		uint8_t  t1_temp;
		uint8_t  t2_temp;
		uint8_t  t3_temp;
		uint8_t  t4_temp;
		uint8_t  t5_temp;
		uint8_t  temp_hys;
		uint16_t t1_t2_chg_voltage;
		uint16_t t2_t3_chg_voltage;
		uint16_t t3_t4_chg_voltage;
		uint16_t t4_t5_chg_voltage;
		uint8_t  t1_t2_chg_current;
		uint8_t  t2_t3_chg_current;
		uint8_t  t3_t4_chg_current;
		uint8_t  t4_t5_chg_current;
	};

	bq_subclass(jeita);

	bq_subclass_jeita.data_structure.t1_temp            = BQ27742_T1_TEMP;
	bq_subclass_jeita.data_structure.t2_temp            = BQ27742_T2_TEMP;
	bq_subclass_jeita.data_structure.t3_temp            = BQ27742_T3_TEMP;
	bq_subclass_jeita.data_structure.t4_temp            = BQ27742_T4_TEMP;
	bq_subclass_jeita.data_structure.t5_temp            = BQ27742_T5_TEMP;
	bq_subclass_jeita.data_structure.temp_hys           = BQ27742_TEMP_HYS;
	bq_subclass_jeita.data_structure.t1_t2_chg_voltage  = BQ27742_T1_T2_CHG_VOLTAGE;
	bq_subclass_jeita.data_structure.t2_t3_chg_voltage  = BQ27742_T2_T3_CHG_VOLTAGE;
	bq_subclass_jeita.data_structure.t3_t4_chg_voltage  = BQ27742_T3_T4_CHG_VOLTAGE;
	bq_subclass_jeita.data_structure.t4_t5_chg_voltage  = BQ27742_T4_T5_CHG_VOLTAGE;
	bq_subclass_jeita.data_structure.t1_t2_chg_current  = BQ27742_T1_T2_CHG_CURRENT;
	bq_subclass_jeita.data_structure.t2_t3_chg_current  = BQ27742_T2_T3_CHG_CURRENT;
	bq_subclass_jeita.data_structure.t3_t4_chg_current  = BQ27742_T3_T4_CHG_CURRENT;
	bq_subclass_jeita.data_structure.t4_t5_chg_current  = BQ27742_T4_T5_CHG_CURRENT;

	bq_subclass_jeita.data_structure.t1_temp            = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t1_temp);
	bq_subclass_jeita.data_structure.t2_temp            = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t2_temp);
	bq_subclass_jeita.data_structure.t3_temp            = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t3_temp);
	bq_subclass_jeita.data_structure.t4_temp            = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t4_temp);
	bq_subclass_jeita.data_structure.t5_temp            = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t5_temp);
	bq_subclass_jeita.data_structure.temp_hys           = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.temp_hys);
	bq_subclass_jeita.data_structure.t1_t2_chg_voltage  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t1_t2_chg_voltage);
	bq_subclass_jeita.data_structure.t2_t3_chg_voltage  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t2_t3_chg_voltage);
	bq_subclass_jeita.data_structure.t3_t4_chg_voltage  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t3_t4_chg_voltage);
	bq_subclass_jeita.data_structure.t4_t5_chg_voltage  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t4_t5_chg_voltage);
	bq_subclass_jeita.data_structure.t1_t2_chg_current  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t1_t2_chg_current);
	bq_subclass_jeita.data_structure.t2_t3_chg_current  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t2_t3_chg_current);
	bq_subclass_jeita.data_structure.t3_t4_chg_current  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t3_t4_chg_current);
	bq_subclass_jeita.data_structure.t4_t5_chg_current  = ENSURE_ENDIANNESS(bq_subclass_jeita.data_structure.t4_t5_chg_current);

	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_jeita.raw, sizeof(bq_subclass_jeita));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_data()
{
	//trzeba tak zrobić, bo jest przerwa w bloku
	bq27742_data_flash_write(48,0,0,BQ27742_DESIGN_VOLTAGE ,2);
	WDT_RR();

	struct __attribute__((packed)) data{
		uint16_t cycle_count;
		uint16_t cc_threshold;
		uint16_t design_capacity;
		uint16_t design_energy;
		uint16_t soh_load_i;
		uint8_t	 tdd_soh_percent;
		uint16_t isd_current;
		uint8_t  isd_i_filter;
		uint8_t  min_isd_time;
		uint8_t  design_energy_scale;
	};

	bq_subclass(data);

	bq_subclass_data.data_structure.cycle_count          = BQ27742_CYCLE_COUNT_;
	bq_subclass_data.data_structure.cc_threshold         = BQ27742_CC_THRESHOLD;
	bq_subclass_data.data_structure.design_capacity      = BQ27742_DESIGN_CAPACITY_;
	bq_subclass_data.data_structure.design_energy        = BQ27742_DESIGN_ENERGY;
	bq_subclass_data.data_structure.soh_load_i           = BQ27742_SOH_LOAD_I;
	bq_subclass_data.data_structure.tdd_soh_percent		 = BQ27742_TDD_SOH_PERCENT;
	bq_subclass_data.data_structure.isd_current          = BQ27742_ISD_CURRENT;
	bq_subclass_data.data_structure.isd_i_filter         = BQ27742_ISD_I_FILTER;
	bq_subclass_data.data_structure.min_isd_time         = BQ27742_MIN_ISD_TIME;
	bq_subclass_data.data_structure.design_energy_scale  = BQ27742_DESIGN_ENERGY_SCALE;

	bq_subclass_data.data_structure.cycle_count          = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.cycle_count);
	bq_subclass_data.data_structure.cc_threshold         = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.cc_threshold);
	bq_subclass_data.data_structure.design_capacity      = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.design_capacity);
	bq_subclass_data.data_structure.design_energy        = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.design_energy);
	bq_subclass_data.data_structure.soh_load_i           = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.soh_load_i);
	bq_subclass_data.data_structure.tdd_soh_percent      = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.tdd_soh_percent);
	bq_subclass_data.data_structure.isd_current          = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.isd_current);
	bq_subclass_data.data_structure.isd_i_filter         = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.isd_i_filter);
	bq_subclass_data.data_structure.min_isd_time         = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.min_isd_time);
	bq_subclass_data.data_structure.design_energy_scale  = ENSURE_ENDIANNESS(bq_subclass_data.data_structure.design_energy_scale);

	uint8_t subclass_id=48;
	uint8_t offset=8;
	uint8_t flash_block=0;

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_data.raw, sizeof(bq_subclass_data));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_discharge()
{
	uint8_t subclass_id=49;

	struct __attribute__((packed)) discharge{
		uint16_t soc1_set_threshold;
		uint16_t soc1_clear_threshold;
		uint16_t socf_set_threshold;
		uint16_t socf_clear_threshold;
		uint16_t bl_set_volt_threshold;
		uint8_t  bl_set_volt_time;
		uint16_t bl_clear_volt_threshold;
		uint16_t bh_set_volt_threshold;
		uint8_t  bh_volt_time;
		uint16_t bh_clear_volt_threshold;
	};

	bq_subclass(discharge);

	bq_subclass_discharge.data_structure.soc1_set_threshold       = BQ27742_SOC1_SET_THRESHOLD;
	bq_subclass_discharge.data_structure.soc1_clear_threshold     = BQ27742_SOC1_CLEAR_THRESHOLD;
	bq_subclass_discharge.data_structure.socf_set_threshold       = BQ27742_SOCF_SET_THRESHOLD;
	bq_subclass_discharge.data_structure.socf_clear_threshold     = BQ27742_SOCF_CLEAR_THRESHOLD;
	bq_subclass_discharge.data_structure.bl_set_volt_threshold    = BQ27742_BL_SET_VOLT_THRESHOLD;
	bq_subclass_discharge.data_structure.bl_set_volt_time         = BQ27742_BL_SET_VOLT_TIME;
	bq_subclass_discharge.data_structure.bl_clear_volt_threshold  = BQ27742_BL_CLEAR_VOLT_THRESHOLD;
	bq_subclass_discharge.data_structure.bh_set_volt_threshold    = BQ27742_BH_SET_VOLT_THRESHOLD;
	bq_subclass_discharge.data_structure.bh_volt_time             = BQ27742_BH_VOLT_TIME;
	bq_subclass_discharge.data_structure.bh_clear_volt_threshold  = BQ27742_BH_CLEAR_VOLT_THRESHOLD;

	bq_subclass_discharge.data_structure.soc1_set_threshold       = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.soc1_set_threshold);
	bq_subclass_discharge.data_structure.soc1_clear_threshold     = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.soc1_clear_threshold);
	bq_subclass_discharge.data_structure.socf_set_threshold       = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.socf_set_threshold);
	bq_subclass_discharge.data_structure.socf_clear_threshold     = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.socf_clear_threshold);
	bq_subclass_discharge.data_structure.bl_set_volt_threshold    = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.bl_set_volt_threshold);
	bq_subclass_discharge.data_structure.bl_set_volt_time         = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.bl_set_volt_time);
	bq_subclass_discharge.data_structure.bl_clear_volt_threshold  = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.bl_clear_volt_threshold);
	bq_subclass_discharge.data_structure.bh_set_volt_threshold    = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.bh_set_volt_threshold);
	bq_subclass_discharge.data_structure.bh_volt_time             = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.bh_volt_time);
	bq_subclass_discharge.data_structure.bh_clear_volt_threshold  = ENSURE_ENDIANNESS(bq_subclass_discharge.data_structure.bh_clear_volt_threshold);

	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_discharge.raw, sizeof(bq_subclass_discharge));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_manufacturer_data()
//TODO rewrite using subclass
{
	uint8_t subclass_id=56;

        struct __attribute__((packed)) manu_data{
          uint16_t pack_lot_code;
          uint16_t pcb_lot_code;
          uint16_t firmware_version;
          uint16_t hardware_revision;
          uint16_t cell_revision;
          uint16_t df_config_version;
        };

        bq_subclass(manu_data);

	bq_subclass_manu_data.data_structure.pack_lot_code      = BQ27742_PACK_LOT_CODE;
	bq_subclass_manu_data.data_structure.pcb_lot_code       = BQ27742_PCB_LOT_CODE;
	bq_subclass_manu_data.data_structure.firmware_version   = BQ27742_FIRMWARE_VERSION;
	bq_subclass_manu_data.data_structure.hardware_revision  = BQ27742_HARDWARE_REVISION;
	bq_subclass_manu_data.data_structure.cell_revision      = BQ27742_CELL_REVISION;
	bq_subclass_manu_data.data_structure.df_config_version  = BQ27742_DF_CONFIG_VERSION;

	bq_subclass_manu_data.data_structure.pack_lot_code      = ENSURE_ENDIANNESS(bq_subclass_manu_data.data_structure.pack_lot_code);
	bq_subclass_manu_data.data_structure.pcb_lot_code       = ENSURE_ENDIANNESS(bq_subclass_manu_data.data_structure.pcb_lot_code);
	bq_subclass_manu_data.data_structure.firmware_version   = ENSURE_ENDIANNESS(bq_subclass_manu_data.data_structure.firmware_version);
	bq_subclass_manu_data.data_structure.hardware_revision  = ENSURE_ENDIANNESS(bq_subclass_manu_data.data_structure.hardware_revision);
	bq_subclass_manu_data.data_structure.cell_revision      = ENSURE_ENDIANNESS(bq_subclass_manu_data.data_structure.cell_revision);
	bq_subclass_manu_data.data_structure.df_config_version  = ENSURE_ENDIANNESS(bq_subclass_manu_data.data_structure.df_config_version);

	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_manu_data.raw, sizeof(bq_subclass_manu_data));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_integrity_data()
{
	uint8_t subclass_id=57;
	uint8_t flash_block=0;
	uint8_t offset=6;

	struct __attribute__((packed)) integrity_data{
		uint16_t all_df_checksum;
		uint16_t static_chem_df_checksum;
		uint16_t static_df_checksum;
		uint16_t prot_checksum;
	};

	bq_subclass(integrity_data);

	bq_subclass_integrity_data.data_structure.all_df_checksum          = BQ27742_ALL_DF_CHECKSUM;
	bq_subclass_integrity_data.data_structure.static_chem_df_checksum  = BQ27742_STATIC_CHEM_DF_CHECKSUM;
	bq_subclass_integrity_data.data_structure.static_df_checksum       = BQ27742_STATIC_DF_CHECKSUM;
	bq_subclass_integrity_data.data_structure.prot_checksum            = BQ27742_PROT_CHECKSUM;

	bq_subclass_integrity_data.data_structure.all_df_checksum          = ENSURE_ENDIANNESS(bq_subclass_integrity_data.data_structure.all_df_checksum);
	bq_subclass_integrity_data.data_structure.static_chem_df_checksum  = ENSURE_ENDIANNESS(bq_subclass_integrity_data.data_structure.static_chem_df_checksum);
	bq_subclass_integrity_data.data_structure.static_df_checksum       = ENSURE_ENDIANNESS(bq_subclass_integrity_data.data_structure.static_df_checksum);
	bq_subclass_integrity_data.data_structure.prot_checksum            = ENSURE_ENDIANNESS(bq_subclass_integrity_data.data_structure.prot_checksum);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_integrity_data.raw, sizeof(bq_subclass_integrity_data));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_lifetime_data()
{
	uint8_t subclass_id=59;
	uint8_t flash_block=0;
	uint8_t offset=0;

	struct __attribute__((packed)) lifetime_data{
		uint16_t lifetime_max_temp;
		uint16_t lifetime_min_temp;
		uint16_t lifetime_max_pack_voltage;
		uint16_t lifetime_min_pack_voltage;
		uint16_t lifetime_max_chg_current;
		uint16_t lifetime_max_dsg_current;
	};

	bq_subclass(lifetime_data);

	bq_subclass_lifetime_data.data_structure.lifetime_max_temp          = BQ27742_LIFETIME_MAX_TEMP;
	bq_subclass_lifetime_data.data_structure.lifetime_min_temp          = BQ27742_LIFETIME_MIN_TEMP;
	bq_subclass_lifetime_data.data_structure.lifetime_max_pack_voltage  = BQ27742_LIFETIME_MAX_PACK_VOLTAGE;
	bq_subclass_lifetime_data.data_structure.lifetime_min_pack_voltage  = BQ27742_LIFETIME_MIN_PACK_VOLTAGE;
	bq_subclass_lifetime_data.data_structure.lifetime_max_chg_current   = BQ27742_LIFETIME_MAX_CHG_CURRENT;
	bq_subclass_lifetime_data.data_structure.lifetime_max_dsg_current   = BQ27742_LIFETIME_MAX_DSG_CURRENT;

	bq_subclass_lifetime_data.data_structure.lifetime_max_temp          = ENSURE_ENDIANNESS(bq_subclass_lifetime_data.data_structure.lifetime_max_temp);
	bq_subclass_lifetime_data.data_structure.lifetime_min_temp          = ENSURE_ENDIANNESS(bq_subclass_lifetime_data.data_structure.lifetime_min_temp);
	bq_subclass_lifetime_data.data_structure.lifetime_max_pack_voltage  = ENSURE_ENDIANNESS(bq_subclass_lifetime_data.data_structure.lifetime_max_pack_voltage);
	bq_subclass_lifetime_data.data_structure.lifetime_min_pack_voltage  = ENSURE_ENDIANNESS(bq_subclass_lifetime_data.data_structure.lifetime_min_pack_voltage);
	bq_subclass_lifetime_data.data_structure.lifetime_max_chg_current   = ENSURE_ENDIANNESS(bq_subclass_lifetime_data.data_structure.lifetime_max_chg_current);
	bq_subclass_lifetime_data.data_structure.lifetime_max_dsg_current   = ENSURE_ENDIANNESS(bq_subclass_lifetime_data.data_structure.lifetime_max_dsg_current);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_lifetime_data.raw, sizeof(bq_subclass_lifetime_data));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_lifetime_temp_samples()
{
	uint8_t subclass_id=60;
	uint8_t flash_block=0;
	uint8_t offset=0;

	struct __attribute__((packed)) lifetime_temp_samples{
		uint16_t lt_flash_cnt;
		uint8_t  lt_afe_status;
	};

	bq_subclass(lifetime_temp_samples);

	bq_subclass_lifetime_temp_samples.data_structure.lt_flash_cnt   = BQ27742_LT_FLASH_CNT;
	bq_subclass_lifetime_temp_samples.data_structure.lt_afe_status  = BQ27742_LT_AFE_STATUS;

	bq_subclass_lifetime_temp_samples.data_structure.lt_flash_cnt   = ENSURE_ENDIANNESS(bq_subclass_lifetime_temp_samples.data_structure.lt_flash_cnt);
	bq_subclass_lifetime_temp_samples.data_structure.lt_afe_status  = ENSURE_ENDIANNESS(bq_subclass_lifetime_temp_samples.data_structure.lt_afe_status);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_lifetime_temp_samples.raw, sizeof(bq_subclass_lifetime_temp_samples));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_registers()
{
	uint8_t subclass_id=64;
	uint8_t flash_block=0;
	uint8_t offset=0;

	struct __attribute__((packed)) registers{
		uint16_t pack_configuration;
		uint8_t  pack_configuration_b;
		uint8_t  pack_configuration_c;
		uint8_t  pack_configuration_d;
		uint8_t  prot_oc_config;
		uint8_t  prot_ov_config;
	};

	bq_subclass(registers);

	bq_subclass_registers.data_structure.pack_configuration    = BQ27742_PACK_CONFIGURATION;
	bq_subclass_registers.data_structure.pack_configuration_b  = BQ27742_PACK_CONFIGURATION_B;
	bq_subclass_registers.data_structure.pack_configuration_c  = BQ27742_PACK_CONFIGURATION_C;
	bq_subclass_registers.data_structure.pack_configuration_d  = BQ27742_PACK_CONFIGURATION_D;
	bq_subclass_registers.data_structure.prot_oc_config        = BQ27742_PROT_OC_CONFIG;
	bq_subclass_registers.data_structure.prot_ov_config        = BQ27742_PROT_OV_CONFIG;

	bq_subclass_registers.data_structure.pack_configuration    = ENSURE_ENDIANNESS(bq_subclass_registers.data_structure.pack_configuration);
	bq_subclass_registers.data_structure.pack_configuration_b  = ENSURE_ENDIANNESS(bq_subclass_registers.data_structure.pack_configuration_b);
	bq_subclass_registers.data_structure.pack_configuration_c  = ENSURE_ENDIANNESS(bq_subclass_registers.data_structure.pack_configuration_c);
	bq_subclass_registers.data_structure.pack_configuration_d  = ENSURE_ENDIANNESS(bq_subclass_registers.data_structure.pack_configuration_d);
	bq_subclass_registers.data_structure.prot_oc_config        = ENSURE_ENDIANNESS(bq_subclass_registers.data_structure.prot_oc_config);
	bq_subclass_registers.data_structure.prot_ov_config        = ENSURE_ENDIANNESS(bq_subclass_registers.data_structure.prot_ov_config);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_registers.raw, sizeof(bq_subclass_registers));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_lifetime_resolution()
{
	uint8_t subclass_id=66;
	uint8_t flash_block=0;
	uint8_t offset=0;

        struct __attribute__((packed)) lifetime_res{
          uint8_t lt_temp_res;
          uint8_t lt_v_res;
          uint8_t lt_cur_res;
          uint16_t lt_update_time;
          uint16_t flash_update_ok_voltage;
        };

	bq_subclass(lifetime_res);

	bq_subclass_lifetime_res.data_structure.lt_temp_res             = BQ27742_LT_TEMP_RES;
	bq_subclass_lifetime_res.data_structure.lt_v_res                = BQ27742_LT_V_RES;
	bq_subclass_lifetime_res.data_structure.lt_cur_res              = BQ27742_LT_CUR_RES;
	bq_subclass_lifetime_res.data_structure.lt_update_time          = BQ27742_LT_UPDATE_TIME;
	bq_subclass_lifetime_res.data_structure.flash_update_ok_voltage = BQ27742_FLASH_UPDATE_OK_VOLTAGE;

	bq_subclass_lifetime_res.data_structure.lt_temp_res             = ENSURE_ENDIANNESS(bq_subclass_lifetime_res.data_structure.lt_temp_res);
	bq_subclass_lifetime_res.data_structure.lt_v_res                = ENSURE_ENDIANNESS(bq_subclass_lifetime_res.data_structure.lt_v_res);
	bq_subclass_lifetime_res.data_structure.lt_cur_res              = ENSURE_ENDIANNESS(bq_subclass_lifetime_res.data_structure.lt_cur_res);
	bq_subclass_lifetime_res.data_structure.lt_update_time          = ENSURE_ENDIANNESS(bq_subclass_lifetime_res.data_structure.lt_update_time);
	bq_subclass_lifetime_res.data_structure.flash_update_ok_voltage = ENSURE_ENDIANNESS(bq_subclass_lifetime_res.data_structure.flash_update_ok_voltage);


	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_lifetime_res.raw, sizeof(bq_subclass_lifetime_res));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_power()
{
	uint8_t subclass_id=68;
	uint8_t flash_block=0;
	uint8_t offset=0;

        struct __attribute__((packed)) power{
          uint16_t flash_update_ok_voltage;
          uint16_t sleep_current;
        };

        bq_subclass(power);

        bq_subclass_power.data_structure.flash_update_ok_voltage  = BQ27742_FLASH_UPDATE_OK_VOLTAGE;
        bq_subclass_power.data_structure.sleep_current            = BQ27742_SLEEP_CURRENT;

        bq_subclass_power.data_structure.flash_update_ok_voltage  = ENSURE_ENDIANNESS(bq_subclass_power.data_structure.flash_update_ok_voltage);
        bq_subclass_power.data_structure.sleep_current            = ENSURE_ENDIANNESS(bq_subclass_power.data_structure.sleep_current);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_power.raw, sizeof(bq_subclass_power));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();

	bq27742_data_flash_write(68,0,10,BQ27742_SHUTDOWN_V ,2); //ok
	bq27742_data_flash_write(68,0,12,BQ27742_FS_WAIT ,1); //ok

}

void bq27742_program_flash_subclass_manufacturer_info1()
//TODO rewrite using subclass
{
	const uint8_t write_value_size=32;
	uint8_t subclass_id=58;
	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t write_value [write_value_size];
			write_value [0] = (uint8_t) (BQ27742_BLOCK_A_0);
			write_value [1] = (uint8_t) (BQ27742_BLOCK_A_1);
			write_value [2] = (uint8_t) (BQ27742_BLOCK_A_2);
			write_value [3] = (uint8_t) (BQ27742_BLOCK_A_3);
			write_value [4] = (uint8_t) (BQ27742_BLOCK_A_4);
			write_value [5] = (uint8_t) (BQ27742_BLOCK_A_5);
			write_value [6] = (uint8_t) (BQ27742_BLOCK_A_6);
			write_value [7] = (uint8_t) (BQ27742_BLOCK_A_7);
			write_value [8] = (uint8_t) (BQ27742_BLOCK_A_8);
			write_value [9] = (uint8_t) (BQ27742_BLOCK_A_9);
			write_value [10] = (uint8_t) (BQ27742_BLOCK_A_10);
			write_value [11] = (uint8_t) (BQ27742_BLOCK_A_11);
			write_value [12] = (uint8_t) (BQ27742_BLOCK_A_12);
			write_value [13] = (uint8_t) (BQ27742_BLOCK_A_13);
			write_value [14] = (uint8_t) (BQ27742_BLOCK_A_14);
			write_value [15] = (uint8_t) (BQ27742_BLOCK_A_15);
			write_value [16] = (uint8_t) (BQ27742_BLOCK_A_16);
			write_value [17] = (uint8_t) (BQ27742_BLOCK_A_17);
			write_value [18] = (uint8_t) (BQ27742_BLOCK_A_18);
			write_value [19] = (uint8_t) (BQ27742_BLOCK_A_19);
			write_value [20] = (uint8_t) (BQ27742_BLOCK_A_20);
			write_value [21] = (uint8_t) (BQ27742_BLOCK_A_21);
			write_value [22] = (uint8_t) (BQ27742_BLOCK_A_22);
			write_value [23] = (uint8_t) (BQ27742_BLOCK_A_23);
			write_value [24] = (uint8_t) (BQ27742_BLOCK_A_24);
			write_value [25] = (uint8_t) (BQ27742_BLOCK_A_25);
			write_value [26] = (uint8_t) (BQ27742_BLOCK_A_26);
			write_value [27] = (uint8_t) (BQ27742_BLOCK_A_27);
			write_value [28] = (uint8_t) (BQ27742_BLOCK_A_28);
			write_value [29] = (uint8_t) (BQ27742_BLOCK_A_29);
			write_value [30] = (uint8_t) (BQ27742_BLOCK_A_30);
			write_value [31] = (uint8_t) (BQ27742_BLOCK_A_31);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, write_value, write_value_size);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();

}

void bq27742_program_flash_subclass_manufacturer_info2()
//TODO rewrite using subclass
{
	const uint8_t write_value_size=32;
	uint8_t subclass_id=58;
	uint8_t flash_block=1;
	uint8_t offset=0;

	uint8_t write_value [write_value_size];
			write_value [0] = (uint8_t) (BQ27742_BLOCK_B_0);
			write_value [1] = (uint8_t) (BQ27742_BLOCK_B_1);
			write_value [2] = (uint8_t) (BQ27742_BLOCK_B_2);
			write_value [3] = (uint8_t) (BQ27742_BLOCK_B_3);
			write_value [4] = (uint8_t) (BQ27742_BLOCK_B_4);
			write_value [5] = (uint8_t) (BQ27742_BLOCK_B_5);
			write_value [6] = (uint8_t) (BQ27742_BLOCK_B_6);
			write_value [7] = (uint8_t) (BQ27742_BLOCK_B_7);
			write_value [8] = (uint8_t) (BQ27742_BLOCK_B_8);
			write_value [9] = (uint8_t) (BQ27742_BLOCK_B_9);
			write_value [10] = (uint8_t) (BQ27742_BLOCK_B_10);
			write_value [11] = (uint8_t) (BQ27742_BLOCK_B_11);
			write_value [12] = (uint8_t) (BQ27742_BLOCK_B_12);
			write_value [13] = (uint8_t) (BQ27742_BLOCK_B_13);
			write_value [14] = (uint8_t) (BQ27742_BLOCK_B_14);
			write_value [15] = (uint8_t) (BQ27742_BLOCK_B_15);
			write_value [16] = (uint8_t) (BQ27742_BLOCK_B_16);
			write_value [17] = (uint8_t) (BQ27742_BLOCK_B_17);
			write_value [18] = (uint8_t) (BQ27742_BLOCK_B_18);
			write_value [19] = (uint8_t) (BQ27742_BLOCK_B_19);
			write_value [20] = (uint8_t) (BQ27742_BLOCK_B_20);
			write_value [21] = (uint8_t) (BQ27742_BLOCK_B_21);
			write_value [22] = (uint8_t) (BQ27742_BLOCK_B_22);
			write_value [23] = (uint8_t) (BQ27742_BLOCK_B_23);
			write_value [24] = (uint8_t) (BQ27742_BLOCK_B_24);
			write_value [25] = (uint8_t) (BQ27742_BLOCK_B_25);
			write_value [26] = (uint8_t) (BQ27742_BLOCK_B_26);
			write_value [27] = (uint8_t) (BQ27742_BLOCK_B_27);
			write_value [28] = (uint8_t) (BQ27742_BLOCK_B_28);
			write_value [29] = (uint8_t) (BQ27742_BLOCK_B_29);
			write_value [30] = (uint8_t) (BQ27742_BLOCK_B_30);
			write_value [31] = (uint8_t) (BQ27742_BLOCK_B_31);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, write_value, write_value_size);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_if_cfg1()
//TODO rewrite using subclass
{
	bq27742_data_flash_write(80,0,0,BQ27742_LOAD_SELECT ,1);
	WDT_RR();
	bq27742_data_flash_write(80,0,1,BQ27742_LOAD_MODE ,1);
	WDT_RR();
	bq27742_data_flash_write(80,0,17,BQ27742_MAX_RES_FACTOR ,1);
	WDT_RR();
	bq27742_data_flash_write(80,0,18,BQ27742_MIN_RES_FACTOR ,1);
	WDT_RR();
	bq27742_data_flash_write(80,0,20,BQ27742_RA_FILTER ,2);
	WDT_RR();
	bq27742_data_flash_write(80,0,22, BQ27742_RES_V_DROP,2);
	WDT_RR();
	bq27742_data_flash_write(80,1,7, BQ27742_FAST_QMAX_START_DOD_PROCENT,1);
	WDT_RR();
	bq27742_data_flash_write(80,1,8,BQ27742_FAST_QMAX_END_DOD_PROCENT ,1);
	WDT_RR();
	bq27742_data_flash_write(80,1,9,BQ27742_FAST_QMAX_STAR_VOLT_DELTA ,2);
	WDT_RR();
	bq27742_data_flash_write(80,1,11,BQ27742_FAST_QMAX_CURRENT_THRESHOLD ,2);
	WDT_RR();
	bq27742_data_flash_write(80,1,29,BQ27742_QMAX_CAPACITY_ERR ,1);
	WDT_RR();
	bq27742_data_flash_write(80,1,30,BQ27742_MAX_QMAX_CHANGE ,1);
	WDT_RR();
	bq27742_data_flash_write(80,2,0, BQ27742_TERMINATE_VOLTAGE,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,2,BQ27742_TERM_V_DELTA ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,5,BQ27742_RESRELAX_TIME ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,9, BQ27742_USER_RATE_MA,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,11, BQ27742_USER_RATE_PWR,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,13,BQ27742_RESERVE_CAP_MAH ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,20,BQ27742_MAX_DELTAV ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,22,BQ27742_MIN_DELTAV ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,24,BQ27742_MAX_SIM_RATE ,1);
	WDT_RR();
	bq27742_data_flash_write(80,2,25, BQ27742_MIN_SIM_RATE,1);
	WDT_RR();
	bq27742_data_flash_write(80,2,26,BQ27742_RA_MAX_DELTA ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,28,BQ27742_TRACE_RESISTANCE ,2);
	WDT_RR();
	bq27742_data_flash_write(80,2,30,BQ27742_DOWNSTREAM_RESISTANCE ,2);
	WDT_RR();
	bq27742_data_flash_write(80,3,0,BQ27742_QMAX_MAX_DELTA_PROCENT ,1);
	WDT_RR();
	bq27742_data_flash_write(80,3,1,BQ27742_QMAX_BOUND_PROCENT ,1);
	WDT_RR();
	bq27742_data_flash_write(80,3,2,BQ27742_DELTAV_MAX_DELTA ,2);
	WDT_RR();
	bq27742_data_flash_write(80,3,4,BQ27742_MAX_RES_SCALE ,2);
	WDT_RR();
	bq27742_data_flash_write(80,3,6,BQ27742_MIN_RES_SCALE ,2);
	WDT_RR();
	bq27742_data_flash_write(80,3,8,BQ27742_FAST_SCALE_START_SOC ,1);
	WDT_RR();
	bq27742_data_flash_write(80,3,9,BQ27742_FAST_SCALE_LOAD_SELECT ,1);
	WDT_RR();
	bq27742_data_flash_write(80,3,10,BQ27742_CHARGE_HYS_V_SHIFT ,2);
	WDT_RR();
	bq27742_data_flash_write(80,3,12,BQ27742_RASCL_OCV_RST_TEMP_THRESH ,1);
	WDT_RR();
	bq27742_data_flash_write(80,3,13,BQ27742_MAX_ALLOWED_CURRENT,2);
	WDT_RR();
	bq27742_data_flash_write(80,3,15,BQ27742_MAX_CURRENT_PULSE_DURATION,1);
	WDT_RR();
	bq27742_data_flash_write(80,3,16,BQ27742_MAX_CURRENT_INTERRUPT_STEP,2);
	WDT_RR();

}


void bq27742_program_flash_subclass_if_cfg2()
//TODO rewrite using subclass
{
	//////////////////////////////
	bq27742_data_flash_write(80,2,5,BQ27742_RESRELAX_TIME ,2);
	//////////////////
	WDT_RR();


	uint8_t offset=9;
	uint8_t flash_block=2;

        struct __attribute__((packed)) cfg2_1{
          uint16_t user_rate_ma;
          uint16_t user_rate_pwr;
          uint16_t reserve_cap_mah;
        };

        bq_subclass(cfg2_1);

        bq_subclass_cfg2_1.data_structure.user_rate_ma     = BQ27742_USER_RATE_MA;
        bq_subclass_cfg2_1.data_structure.user_rate_pwr    = BQ27742_USER_RATE_PWR;
        bq_subclass_cfg2_1.data_structure.reserve_cap_mah  = BQ27742_RESERVE_CAP_MAH;

        bq_subclass_cfg2_1.data_structure.user_rate_ma     = ENSURE_ENDIANNESS(bq_subclass_cfg2_1.data_structure.user_rate_ma);
        bq_subclass_cfg2_1.data_structure.user_rate_pwr    = ENSURE_ENDIANNESS(bq_subclass_cfg2_1.data_structure.user_rate_pwr);
        bq_subclass_cfg2_1.data_structure.reserve_cap_mah  = ENSURE_ENDIANNESS(bq_subclass_cfg2_1.data_structure.reserve_cap_mah);

	uint8_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;
	uint8_t subclass_id=80;
	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint8_t flash_date_access = 0x00;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_cfg2_1.raw, sizeof(bq_subclass_cfg2_1));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];
	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
//////////////////
	WDT_RR();
	offset=20;
	flash_block=2;

        struct __attribute__((packed)) cfg2_2{
          uint16_t max_deltav;
          uint16_t min_deltav;
          uint8_t max_sim_rate;
          uint8_t min_sim_rate;
          uint16_t ra_max_delta;
          uint16_t trace_resistance;
          uint16_t downstream_resistance;
        };

        bq_subclass(cfg2_2);

        bq_subclass_cfg2_2.data_structure.max_deltav             = BQ27742_MAX_DELTAV;
        bq_subclass_cfg2_2.data_structure.min_deltav             = BQ27742_MIN_DELTAV;
        bq_subclass_cfg2_2.data_structure.max_sim_rate           = BQ27742_MAX_SIM_RATE;
        bq_subclass_cfg2_2.data_structure.min_sim_rate           = BQ27742_MIN_SIM_RATE;
        bq_subclass_cfg2_2.data_structure.ra_max_delta           = BQ27742_RA_MAX_DELTA;
        bq_subclass_cfg2_2.data_structure.trace_resistance       = BQ27742_TRACE_RESISTANCE;
        bq_subclass_cfg2_2.data_structure.downstream_resistance  = BQ27742_DOWNSTREAM_RESISTANCE;

        bq_subclass_cfg2_2.data_structure.max_deltav             = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.max_deltav);
        bq_subclass_cfg2_2.data_structure.min_deltav             = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.min_deltav);
        bq_subclass_cfg2_2.data_structure.max_sim_rate           = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.max_sim_rate);
        bq_subclass_cfg2_2.data_structure.min_sim_rate           = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.min_sim_rate);
        bq_subclass_cfg2_2.data_structure.ra_max_delta           = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.ra_max_delta);
        bq_subclass_cfg2_2.data_structure.trace_resistance       = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.trace_resistance);
        bq_subclass_cfg2_2.data_structure.downstream_resistance  = ENSURE_ENDIANNESS(bq_subclass_cfg2_2.data_structure.downstream_resistance);


	flash_date_sum = 0;
	checksum = 0;
	block_date_address = BQ27742_BLOCK_DATA + offset;
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_cfg2_2.raw, sizeof(bq_subclass_cfg2_2));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];
	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
	//////////////////
	WDT_RR();
	offset=0;
	flash_block=3;

        struct __attribute__((packed)) cfg2_3{
          uint8_t   qmax_max_delta_procent;
          uint8_t   qmax_bound_procent;
          uint16_t  deltav_max_delta;
          uint16_t  max_res_scale;
          uint16_t  min_res_scale;
          uint8_t   fast_scale_start_soc;
          uint8_t   fast_scale_load_select;
          uint16_t  charge_hys_v_shift;
          uint8_t   rascl_ocv_rst_temp_thresh;
          uint16_t  max_allowed_current;
          uint8_t   max_current_pulse_duration;
          uint16_t  max_current_interrupt_step;
        };

        bq_subclass(cfg2_3);

        bq_subclass_cfg2_3.data_structure.qmax_max_delta_procent     = BQ27742_QMAX_MAX_DELTA_PROCENT;
        bq_subclass_cfg2_3.data_structure.qmax_bound_procent         = BQ27742_QMAX_BOUND_PROCENT;
        bq_subclass_cfg2_3.data_structure.deltav_max_delta           = BQ27742_DELTAV_MAX_DELTA;
        bq_subclass_cfg2_3.data_structure.max_res_scale              = BQ27742_MAX_RES_SCALE;
        bq_subclass_cfg2_3.data_structure.min_res_scale              = BQ27742_MIN_RES_SCALE;
        bq_subclass_cfg2_3.data_structure.fast_scale_start_soc       = BQ27742_FAST_SCALE_START_SOC;
        bq_subclass_cfg2_3.data_structure.fast_scale_load_select     = BQ27742_FAST_SCALE_LOAD_SELECT;
        bq_subclass_cfg2_3.data_structure.charge_hys_v_shift         = BQ27742_CHARGE_HYS_V_SHIFT;
        bq_subclass_cfg2_3.data_structure.rascl_ocv_rst_temp_thresh  = BQ27742_RASCL_OCV_RST_TEMP_THRESH;
        bq_subclass_cfg2_3.data_structure.max_allowed_current        = BQ27742_MAX_ALLOWED_CURRENT;
        bq_subclass_cfg2_3.data_structure.max_current_pulse_duration = BQ27742_MAX_CURRENT_PULSE_DURATION;
        bq_subclass_cfg2_3.data_structure.max_current_interrupt_step = BQ27742_MAX_CURRENT_INTERRUPT_STEP;

        bq_subclass_cfg2_3.data_structure.qmax_max_delta_procent     = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.qmax_max_delta_procent);
        bq_subclass_cfg2_3.data_structure.qmax_bound_procent         = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.qmax_bound_procent);
        bq_subclass_cfg2_3.data_structure.deltav_max_delta           = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.deltav_max_delta);
        bq_subclass_cfg2_3.data_structure.max_res_scale              = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.max_res_scale);
        bq_subclass_cfg2_3.data_structure.min_res_scale              = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.min_res_scale);
        bq_subclass_cfg2_3.data_structure.fast_scale_start_soc       = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.fast_scale_start_soc);
        bq_subclass_cfg2_3.data_structure.fast_scale_load_select     = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.fast_scale_load_select);
        bq_subclass_cfg2_3.data_structure.charge_hys_v_shift         = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.charge_hys_v_shift);
        bq_subclass_cfg2_3.data_structure.rascl_ocv_rst_temp_thresh  = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.rascl_ocv_rst_temp_thresh);
        bq_subclass_cfg2_3.data_structure.max_allowed_current        = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.max_allowed_current);
        bq_subclass_cfg2_3.data_structure.max_current_pulse_duration = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.max_current_pulse_duration);
        bq_subclass_cfg2_3.data_structure.max_current_interrupt_step = ENSURE_ENDIANNESS(bq_subclass_cfg2_3.data_structure.max_current_interrupt_step);

	flash_date_sum = 0;
	checksum = 0;
	block_date_address = BQ27742_BLOCK_DATA + offset;
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_cfg2_3.raw, sizeof(bq_subclass_cfg2_3));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];
	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
	return;
}

void bq27742_program_flash_subclass_current_thresholds()
  //TODO rewrite using subclass
{
  uint8_t subclass_id=81;
  uint8_t flash_block=0;
  uint8_t offset=0;

  struct __attribute__((packed)) current_thresholds{
    uint16_t dsg_current_threshold;
    uint16_t chg_current_threshold;
    uint16_t quit_current;
    uint16_t dsg_relax_time;
    uint8_t chg_relax_time;
    uint8_t quit_relax_time;
    uint16_t max_ir_correct;
  };

  bq_subclass(current_thresholds);

  bq_subclass_current_thresholds.data_structure.dsg_current_threshold = BQ27742_DSG_CURRENT_THRESHOLD;
  bq_subclass_current_thresholds.data_structure.chg_current_threshold = BQ27742_CHG_CURRENT_THRESHOLD;
  bq_subclass_current_thresholds.data_structure.quit_current          = BQ27742_QUIT_CURRENT;
  bq_subclass_current_thresholds.data_structure.dsg_relax_time        = BQ27742_DSG_RELAX_TIME;
  bq_subclass_current_thresholds.data_structure.chg_relax_time        = BQ27742_CHG_RELAX_TIME;
  bq_subclass_current_thresholds.data_structure.quit_current          = BQ27742_QUIT_RELAX_TIME;
  bq_subclass_current_thresholds.data_structure.max_ir_correct        = BQ27742_MAX_IR_CORRECT;

  bq_subclass_current_thresholds.data_structure.dsg_current_threshold = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.dsg_current_threshold);
  bq_subclass_current_thresholds.data_structure.chg_current_threshold = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.chg_current_threshold);
  bq_subclass_current_thresholds.data_structure.quit_current          = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.quit_current);
  bq_subclass_current_thresholds.data_structure.dsg_relax_time        = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.dsg_relax_time);
  bq_subclass_current_thresholds.data_structure.chg_relax_time        = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.chg_relax_time);
  bq_subclass_current_thresholds.data_structure.quit_current          = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.quit_current);
  bq_subclass_current_thresholds.data_structure.max_ir_correct        = ENSURE_ENDIANNESS(bq_subclass_current_thresholds.data_structure.max_ir_correct);

  uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
  uint32_t flash_date_sum = 0;
  uint8_t checksum = 0;
  uint8_t flash_date_access = 0x00;
  uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_current_thresholds.raw, sizeof(bq_subclass_current_thresholds));
  TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
  WDT_RR();
  for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
    flash_date_sum +=reg_value[i];

  checksum = 255 - (uint8_t)(flash_date_sum);
  TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
  ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
  WDT_RR();
}

void bq27742_program_flash_subclass_state()
{
	uint8_t subclass_id=82;
	uint8_t flash_block=0;
	uint8_t offset=0;

	struct __attribute__((packed)) state{
		uint16_t qmax_cell_0;
		uint8_t  update_status;
		uint16_t v_at_chg_term;
		uint16_t avg_i_last_run;
		uint16_t avg_p_last_run;
		uint16_t delta_voltage;
		uint16_t t_rise;
		uint16_t t_time_constant;
	};

	bq_subclass(state);

	bq_subclass_state.data_structure.qmax_cell_0      = BQ27742_QMAX_CELL_0;
	bq_subclass_state.data_structure.update_status    = BQ27742_UPDATE_STATUS;
	bq_subclass_state.data_structure.v_at_chg_term    = BQ27742_V_AT_CHG_TERM;
	bq_subclass_state.data_structure.avg_i_last_run   = BQ27742_AVG_I_LAST_RUN;
	bq_subclass_state.data_structure.avg_p_last_run   = BQ27742_AVG_P_LAST_RUN;
	bq_subclass_state.data_structure.delta_voltage    = BQ27742_DELTA_VOLTAGE;
	bq_subclass_state.data_structure.t_rise           = BQ27742_T_RISE;
	bq_subclass_state.data_structure.t_time_constant  = BQ27742_T_TIME_CONSTANT;

	bq_subclass_state.data_structure.qmax_cell_0      = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.qmax_cell_0);
	bq_subclass_state.data_structure.update_status    = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.update_status);
	bq_subclass_state.data_structure.v_at_chg_term    = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.v_at_chg_term);
	bq_subclass_state.data_structure.avg_i_last_run   = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.avg_i_last_run);
	bq_subclass_state.data_structure.avg_p_last_run   = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.avg_p_last_run);
	bq_subclass_state.data_structure.delta_voltage    = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.delta_voltage);
	bq_subclass_state.data_structure.t_rise           = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.t_rise);
	bq_subclass_state.data_structure.t_time_constant  = ENSURE_ENDIANNESS(bq_subclass_state.data_structure.t_time_constant);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_state.raw, sizeof(bq_subclass_state));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_R_a0()
{
	uint8_t subclass_id=88;
	uint8_t flash_block=0;
	uint8_t offset=0;

	struct __attribute__((packed)) r_a0{
		uint16_t cell0_r_a_flag;
		uint16_t cell0_r_a_0;
		uint16_t cell0_r_a_1;
		uint16_t cell0_r_a_2;
		uint16_t cell0_r_a_3;
		uint16_t cell0_r_a_4;
		uint16_t cell0_r_a_5;
		uint16_t cell0_r_a_6;
		uint16_t cell0_r_a_7;
		uint16_t cell0_r_a_8;
		uint16_t cell0_r_a_9;
		uint16_t cell0_r_a_10;
		uint16_t cell0_r_a_11;
		uint16_t cell0_r_a_12;
		uint16_t cell0_r_a_13;
		uint16_t cell0_r_a_14;
	};

	bq_subclass(r_a0);

	bq_subclass_r_a0.data_structure.cell0_r_a_flag  = BQ27742_CELL0_R_A_FLAG;
	bq_subclass_r_a0.data_structure.cell0_r_a_0     = BQ27742_CELL0_R_A_0;
	bq_subclass_r_a0.data_structure.cell0_r_a_1     = BQ27742_CELL0_R_A_1;
	bq_subclass_r_a0.data_structure.cell0_r_a_2     = BQ27742_CELL0_R_A_2;
	bq_subclass_r_a0.data_structure.cell0_r_a_3     = BQ27742_CELL0_R_A_3;
	bq_subclass_r_a0.data_structure.cell0_r_a_4     = BQ27742_CELL0_R_A_4;
	bq_subclass_r_a0.data_structure.cell0_r_a_5     = BQ27742_CELL0_R_A_5;
	bq_subclass_r_a0.data_structure.cell0_r_a_6     = BQ27742_CELL0_R_A_6;
	bq_subclass_r_a0.data_structure.cell0_r_a_7     = BQ27742_CELL0_R_A_7;
	bq_subclass_r_a0.data_structure.cell0_r_a_8     = BQ27742_CELL0_R_A_8;
	bq_subclass_r_a0.data_structure.cell0_r_a_9     = BQ27742_CELL0_R_A_9;
	bq_subclass_r_a0.data_structure.cell0_r_a_10    = BQ27742_CELL0_R_A_10;
	bq_subclass_r_a0.data_structure.cell0_r_a_11    = BQ27742_CELL0_R_A_11;
	bq_subclass_r_a0.data_structure.cell0_r_a_12    = BQ27742_CELL0_R_A_12;
	bq_subclass_r_a0.data_structure.cell0_r_a_13    = BQ27742_CELL0_R_A_13;
	bq_subclass_r_a0.data_structure.cell0_r_a_14    = BQ27742_CELL0_R_A_14;

	bq_subclass_r_a0.data_structure.cell0_r_a_flag  = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_flag);
	bq_subclass_r_a0.data_structure.cell0_r_a_0     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_0);
	bq_subclass_r_a0.data_structure.cell0_r_a_1     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_1);
	bq_subclass_r_a0.data_structure.cell0_r_a_2     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_2);
	bq_subclass_r_a0.data_structure.cell0_r_a_3     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_3);
	bq_subclass_r_a0.data_structure.cell0_r_a_4     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_4);
	bq_subclass_r_a0.data_structure.cell0_r_a_5     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_5);
	bq_subclass_r_a0.data_structure.cell0_r_a_6     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_6);
	bq_subclass_r_a0.data_structure.cell0_r_a_7     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_7);
	bq_subclass_r_a0.data_structure.cell0_r_a_8     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_8);
	bq_subclass_r_a0.data_structure.cell0_r_a_9     = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_9);
	bq_subclass_r_a0.data_structure.cell0_r_a_10    = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_10);
	bq_subclass_r_a0.data_structure.cell0_r_a_11    = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_11);
	bq_subclass_r_a0.data_structure.cell0_r_a_12    = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_12);
	bq_subclass_r_a0.data_structure.cell0_r_a_13    = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_13);
	bq_subclass_r_a0.data_structure.cell0_r_a_14    = ENSURE_ENDIANNESS(bq_subclass_r_a0.data_structure.cell0_r_a_14);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_r_a0.raw, sizeof(bq_subclass_r_a0));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_R_a0X()
{
	uint8_t subclass_id=89;
	uint8_t flash_block=0;
	uint8_t offset=0;

	struct __attribute__((packed)) r_a0x{
		uint16_t xcell0_r_a_flag;
		uint16_t xcell0_r_a_0;
		uint16_t xcell0_r_a_1;
		uint16_t xcell0_r_a_2;
		uint16_t xcell0_r_a_3;
		uint16_t xcell0_r_a_4;
		uint16_t xcell0_r_a_5;
		uint16_t xcell0_r_a_6;
		uint16_t xcell0_r_a_7;
		uint16_t xcell0_r_a_8;
		uint16_t xcell0_r_a_9;
		uint16_t xcell0_r_a_10;
		uint16_t xcell0_r_a_11;
		uint16_t xcell0_r_a_12;
		uint16_t xcell0_r_a_13;
		uint16_t xcell0_r_a_14;
	};

	bq_subclass(r_a0x);

	bq_subclass_r_a0x.data_structure.xcell0_r_a_flag  = BQ27742_XCELL0_R_A_FLAG;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_0     = BQ27742_XCELL0_R_A_0;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_1     = BQ27742_XCELL0_R_A_1;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_2     = BQ27742_XCELL0_R_A_2;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_3     = BQ27742_XCELL0_R_A_3;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_4     = BQ27742_XCELL0_R_A_4;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_5     = BQ27742_XCELL0_R_A_5;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_6     = BQ27742_XCELL0_R_A_6;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_7     = BQ27742_XCELL0_R_A_7;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_8     = BQ27742_XCELL0_R_A_8;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_9     = BQ27742_XCELL0_R_A_9;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_10    = BQ27742_XCELL0_R_A_10;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_11    = BQ27742_XCELL0_R_A_11;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_12    = BQ27742_XCELL0_R_A_12;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_13    = BQ27742_XCELL0_R_A_13;
	bq_subclass_r_a0x.data_structure.xcell0_r_a_14    = BQ27742_XCELL0_R_A_14;

	bq_subclass_r_a0x.data_structure.xcell0_r_a_flag  = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_flag);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_0     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_0);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_1     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_1);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_2     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_2);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_3     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_3);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_4     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_4);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_5     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_5);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_6     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_6);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_7     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_7);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_8     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_8);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_9     = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_9);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_10    = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_10);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_11    = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_11);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_12    = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_12);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_13    = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_13);
	bq_subclass_r_a0x.data_structure.xcell0_r_a_14    = ENSURE_ENDIANNESS(bq_subclass_r_a0x.data_structure.xcell0_r_a_14);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, bq_subclass_r_a0x.raw, sizeof(bq_subclass_r_a0x));
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_current()
//TODO rewrite using subclass
{
	const uint8_t write_value_size=3;
	uint8_t subclass_id=107;
	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t write_value [write_value_size];
			write_value [0] = (uint8_t) (BQ27742_FILTER);
			write_value [1] = (uint8_t) (BQ27742_DEADBAND);
			write_value [2] = (uint8_t) (BQ27742_CC_DEADBAND);

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, write_value, write_value_size);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash_subclass_codes()
//TODO rewrite using subclass
{
	const uint8_t write_value_size=24;
	uint8_t subclass_id=112;
	uint8_t flash_block=0;
	uint8_t offset=0;
	uint8_t write_value [write_value_size];
			write_value [0] = (uint8_t) (BQ27742_SEALED_TO_UNSEALED>>24);
			write_value [1] = (uint8_t) (BQ27742_SEALED_TO_UNSEALED>>16);
			write_value [2] = (uint8_t) (BQ27742_SEALED_TO_UNSEALED>>8);
			write_value [3] = (uint8_t) (BQ27742_SEALED_TO_UNSEALED);
			write_value [4] = (uint8_t) (BQ27742_UNSEALED_TO_FULL>>24);
			write_value [5] = (uint8_t) (BQ27742_UNSEALED_TO_FULL>>16);
			write_value [6] = (uint8_t) (BQ27742_UNSEALED_TO_FULL>>8);
			write_value [7] = (uint8_t) (BQ27742_UNSEALED_TO_FULL);
			write_value [8] = (uint8_t) (BQ27742_AUTHEN_KEY3>>24);
			write_value [9] = (uint8_t) (BQ27742_AUTHEN_KEY3>>16);
			write_value [10] = (uint8_t) (BQ27742_AUTHEN_KEY3>>8);
			write_value [11] = (uint8_t) (BQ27742_AUTHEN_KEY3);
			write_value [12] = (uint8_t) (BQ27742_AUTHEN_KEY2>>24);
			write_value [13] = (uint8_t) (BQ27742_AUTHEN_KEY2>>16);
			write_value [14] = (uint8_t) (BQ27742_AUTHEN_KEY2>>8);
			write_value [15] = (uint8_t) (BQ27742_AUTHEN_KEY2);
			write_value [16] = (uint8_t) (BQ27742_AUTHEN_KEY1>>24);
			write_value [17] = (uint8_t) (BQ27742_AUTHEN_KEY1>>16);
			write_value [18] = (uint8_t) (BQ27742_AUTHEN_KEY1>>8);
			write_value [19] = (uint8_t) (BQ27742_AUTHEN_KEY1);
			write_value [20] = (uint8_t) (BQ27742_AUTHEN_KEY0>>24);
			write_value [21] = (uint8_t) (BQ27742_AUTHEN_KEY0>>16);
			write_value [22] = (uint8_t) (BQ27742_AUTHEN_KEY0>>8);
			write_value [23] = (uint8_t) (BQ27742_AUTHEN_KEY0);


	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, write_value, write_value_size);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();
}

void bq27742_program_flash()
{
	SetBit(GPIO_LEDS_ON);
	ic_delay_ms(5);
	//	//--PROGRAMOWANIE BQ---------------------------------------
	bq27742_unsaled_set();
	RGB_right_set(COLOR_YELLOW, 10);
	bq27742_full_access_set();
	RGB_right_set(COLOR_CYAN, 10);
	bq27742_program_flash_subclass_safety(); 				//ok
	RGB_right_set(COLOR_YELLOW, 10);
	bq27742_program_flash_subclass_charge(); 				//ok
	RGB_right_set(COLOR_CYAN, 10);
	bq27742_program_flash_subclass_charge_termination(); 	//ok
#ifdef 	BQ_PROGRAMM_DFEFAULT
	bq27742_program_flash_subclass_JEITA(); 				//ok
#endif
	bq27742_program_flash_subclass_data(); 					//ok
	RGB_right_set(COLOR_YELLOW, 10);
	bq27742_program_flash_subclass_discharge(); 			//ok
#ifdef 	BQ_PROGRAMM_DFEFAULT
	bq27742_program_flash_subclass_manufacturer_data(); 	//ok
#endif
	bq27742_program_flash_subclass_integrity_data(); 		//ok
	RGB_right_set(COLOR_CYAN, 10);
	bq27742_program_flash_subclass_lifetime_data();			//ok
	RGB_right_set(COLOR_YELLOW, 10);
	bq27742_program_flash_subclass_lifetime_temp_samples();	//ok
	RGB_right_set(COLOR_CYAN, 10);
	bq27742_program_flash_subclass_registers();				// OK
	RGB_right_set(COLOR_YELLOW, 10);
#ifdef 	BQ_PROGRAMM_DFEFAULT
	bq27742_program_flash_subclass_lifetime_resolution();	//ok
	bq27742_program_flash_subclass_power(); 				//OK
	bq27742_program_flash_subclass_manufacturer_info1();	//ok
	bq27742_program_flash_subclass_manufacturer_info2();	//ok
	bq27742_program_flash_subclass_if_cfg1();				//ok, ale bardzo długie
	bq27742_program_flash_subclass_current_thresholds(); 	//ok
#endif
	bq27742_program_flash_subclass_state(); 				//ok
	RGB_right_set(COLOR_CYAN, 10);
	bq27742_data_flash_write(83,0,0,BQ27742_CHEM_ID_ ,2); 	//ok
	RGB_right_set(COLOR_YELLOW, 10);
	bq27742_program_flash_subclass_R_a0(); 					//ok
	RGB_right_set(COLOR_CYAN, 10);
	bq27742_program_flash_subclass_R_a0X(); 				//ok
	RGB_right_set(COLOR_YELLOW, 10);
	bq27742_program_flash_subclass_data_104();				//ok
	RGB_right_set(COLOR_CYAN, 10);
#ifdef 	BQ_PROGRAMM_DFEFAULT
	bq27742_program_flash_subclass_current(); 				//ok
	bq27742_program_flash_subclass_codes();
#endif
	bq27742_sealed_set();
	RGB_right_set(COLOR_BLACK, 10);
	RGB_left_set(COLOR_BLACK, 10);
}

