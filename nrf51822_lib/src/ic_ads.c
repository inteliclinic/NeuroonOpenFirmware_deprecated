/**
 * @file
 * @brief ADS1115 driver methods
 */

#include "ic_ads.h"

#ifdef USE_IC_ADS

#ifndef USE_IC_I2C
	#define USE_IC_I2C
#endif
#include "ic_i2c.h"

uint8_t config_frame[2];
uint8_t convertion_read_frame[2];

/**
 * @fn ads_init ()
 * @brief initialization device
 * @return true if initialization correct
 */
bool ads_init ()
{
	bool error_comm = false;
	uint16_t check_value=0;
	uint8_t check_frame[2] = {0};

	/** Check communication I2C */
	TWI_ReadReg(ADS_TWI_ADDRESS,ADS_LO_THS_REG,check_frame,2);
	check_value = check_frame[0]<<8 | check_frame[1];

	if(check_value == ADS_LO_THS)
		error_comm = true;
	else
		error_comm = false;

	config_frame[0] = ADS_SINGLE_SHOT_CONV | ADS_GAIN_4;
	config_frame[1] = ADS_DATA_RATE_860 | ADS_COMP_QUE_DIS;

	TWI_WriteReg(ADS_TWI_ADDRESS,ADS_CONFIG_REG,config_frame,2);

	return error_comm;
}
void ads_deinit(){
	return;
}

/**
 * @fn ads_power_down ()
 * @brief set device in power down mode
 */
void ads_power_down ()
{
	config_frame[0] = ADS_POWER_DOWN;
	config_frame[1] = 0;
	TWI_WriteReg(ADS_TWI_ADDRESS,ADS_CONFIG_REG,config_frame,2);
}

/**
 * @fn ads_power_up ()
 * @brief set device in power up mode
 */
void ads_power_up(void)
{
	config_frame[0] = ADS_SINGLE_SHOT_CONV | ADS_GAIN_1;
	config_frame[1] = ADS_DATA_RATE_250 | ADS_COMP_QUE_DIS;

	TWI_WriteReg(ADS_TWI_ADDRESS,ADS_CONFIG_REG,config_frame,2);
}

/**
 * @fn ads_get_value ()
 * @brief get conversion value
 * @return conversion value
 */
int16_t ads_get_value()
{
	int16_t convertion_read = 0;

	TWI_ReadReg(ADS_TWI_ADDRESS,ADS_CONVERTION_REG,convertion_read_frame,2);

	//convertion_read_frame[0] ^= 0x80;
	convertion_read = convertion_read_frame[0]<<8 | convertion_read_frame[1];

	return convertion_read;
}

/**
 * @fn ads_change_gain ()
 * @brief change gain value
 * @param new gain value
 * @return true if change correct
 */
bool ads_change_gain(uint8_t new_gain)
{
	uint8_t new_gain_value = 0;
	bool error_write = false;
	bool check_write_config = false;

	if (new_gain == 0) {
		new_gain_value = 0x00;
	}
	else if (new_gain == 1) {
		new_gain_value = ADS_GAIN_1;
	}
	else if (new_gain == 2) {
		new_gain_value = ADS_GAIN_2;
	}
	else if (new_gain == 4) {
		new_gain_value = ADS_GAIN_4;
	}
	else if (new_gain == 8) {
		new_gain_value = ADS_GAIN_8;
	}
	else if (new_gain == 16) {
		new_gain_value = ADS_GAIN_16;
	}
	else {
		return false;
	}

	config_frame[0] = ADS_SINGLE_SHOT_CONV | new_gain_value;

	check_write_config = TWI_WriteReg(ADS_TWI_ADDRESS,ADS_CONFIG_REG,config_frame,2);

	if (check_write_config == 1)
		error_write = false;
	else
		error_write = true;

	return error_write;
}

/**
 * @fn ads_change_data_rate ()
 * @brief change data rate value
 * @param rate code
 * @return true if change correct
 */
bool ads_change_data_rate(uint8_t rate_code)
{
	uint8_t new_rate_value = 0;
	bool error_write = false;
	bool check_write_config = false;

	if (rate_code == 0) {
		new_rate_value = ADS_DATA_RATE_8;
	}
	else if (rate_code == 1) {
		new_rate_value = ADS_DATA_RATE_16;
	}
	else if (rate_code == 2) {
		new_rate_value = ADS_DATA_RATE_32;
	}
	else if (rate_code == 3) {
		new_rate_value = ADS_DATA_RATE_64;
	}
	else if (rate_code == 4) {
		new_rate_value = ADS_DATA_RATE_128;
	}
	else if (rate_code == 5) {
		new_rate_value = ADS_DATA_RATE_250;
	}
	else if (rate_code == 6) {
		new_rate_value = ADS_DATA_RATE_475;
	}
	else if (rate_code == 6) {
		new_rate_value = ADS_DATA_RATE_860;
	}

	else {
		return false;
	}

	config_frame[1] = ADS_COMP_QUE_DIS | new_rate_value;

	check_write_config = TWI_WriteReg(ADS_TWI_ADDRESS,ADS_CONFIG_REG,config_frame,2);

	if (check_write_config == 1)
		error_write = false;
	else
		error_write = true;

	return error_write;
}

#endif
